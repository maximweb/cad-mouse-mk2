#include "config.h"

#include "normalization.h"

namespace {
    void apply_3d_deadzone_coupled(float* pos, float* vel, float deadzone_threshold, float max_value)
    {
        // Calculate the magnitude of the position vector
        float pos_magnitude = sqrt(pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2]);

        if (pos_magnitude < deadzone_threshold) {
            // If within deadzone, zero out both position and velocity
            for (int i = 0; i < 3; ++i) {
                pos[i] = 0.0f;
                vel[i] = 0.0f;
            }
        }
        else {
            // Scale the position and velocity to account for the deadzone
            float scale = (pos_magnitude - deadzone_threshold) / (max_value - deadzone_threshold) / pos_magnitude;
            for (int i = 0; i < 3; ++i) {
                pos[i] *= scale;
                vel[i] *= scale;
            }
        }
    }

    float apply_power(float x, float power)
    {
        // Shortcuts to harness RP2350 floating point hardware capabilities
        // Limited to plus, minus, multiply, divide, and square root operations.
        if (x <= 1e-8f) {
            return 0.0f;
        }
        if (power == 1.0f) {
            return x;
        }
        else if (power == 2.0f) {
            return x * x;
        }
        else if (power == 3.0f) {
            return x * x * x;
        }
        else if (power == 0.5f) {
            return sqrtf(x);
        }
        else {
            // For other powers, use the standard (slow) library function
            return powf(x, power);
        }
    }
}

void Normalization::apply_normalization_deadzone_isolation(float state[12], float return_state[12])
{
    // Per DoF Limits for normalization
    // We allow individual limits, as spring might behave differently for x,y vs z and rx,ry vs rz.
    // Yet for now, we will use the same limits for translation and rotation.
    const float translation_limits_max[3] = {
      NORMALIZATION_X_MAX,
      NORMALIZATION_Y_MAX,
      NORMALIZATION_Z_MAX,
    }; // mm
    const float rotation_limits_max[3] = {
      NORMALIZATION_RX_MAX * (3.14159265f / 180.0f),
      NORMALIZATION_RY_MAX * (3.14159265f / 180.0f),
      NORMALIZATION_RZ_MAX * (3.14159265f / 180.0f),
    }; // radians
    const float translation_limits_min[3] = {
      NORMALIZATION_X_MIN,
      NORMALIZATION_Y_MIN,
      NORMALIZATION_Z_MIN,
    }; // mm
    const float rotation_limits_min[3] = {
      NORMALIZATION_RX_MIN * (3.14159265f / 180.0f),
      NORMALIZATION_RY_MIN * (3.14159265f / 180.0f),
      NORMALIZATION_RZ_MIN * (3.14159265f / 180.0f),
    }; // radians

    // == Scale translation and rotation values relative to their configured limits.
    // Values may temporarily exceed [-1, 1]; the isolation step bounds the final pose.
    // Use the same scaling for velocities so deadzone and isolation gains remain coupled.
    // This makes
    // - x,y,z: mm -> effort%
    // - rx,ry,rz: radians -> effort%
    // - vx,vy,vz: mm/s -> effort%/s
    // - vrx,vry,vrz: radians/s -> effort%/s
    float normalized_state[12];
    for (int i = 0; i < 3; ++i) {
        if (state[i] < 0.0f) {
            normalized_state[i] = state[i] / translation_limits_min[i];         // Translation
            normalized_state[i + 6] = state[i + 6] / translation_limits_min[i]; // Translation Velocity
        }
        else {
            normalized_state[i] = state[i] / translation_limits_max[i];         // Translation
            normalized_state[i + 6] = state[i + 6] / translation_limits_max[i]; // Translation Velocity
        }
        if (state[i + 3] < 0.0f) {
            normalized_state[i + 3] = state[i + 3] / rotation_limits_min[i]; // Rotation
            normalized_state[i + 9] = state[i + 9] / rotation_limits_min[i]; // Rotation Velocity
        }
        else {
            normalized_state[i + 3] = state[i + 3] / rotation_limits_max[i]; // Rotation
            normalized_state[i + 9] = state[i + 9] / rotation_limits_max[i]; // Rotation Velocity
        }
    }

    // == Deadzone
    // Treat translation and rotation separately, as they may have different deadzone thresholds.
    // Use magnitude of translation and rotation vectors to determine if they are within the deadzone.
    // Velocities are also zeroed out if the corresponding translation or rotation is within the deadzone, to prevent drift.
    const float translation_deadzone_threshold = DEADZONE_TRANSLATION_THRESHOLD;
    const float rotation_deadzone_threshold = DEADZONE_ROTATION_THRESHOLD;

    apply_3d_deadzone_coupled(&normalized_state[0], &normalized_state[6], translation_deadzone_threshold, 1.0f);
    apply_3d_deadzone_coupled(&normalized_state[3], &normalized_state[9], rotation_deadzone_threshold, 1.0f);

    // == 6DoF Dominance Isolation
    // The idea is to apply power of X (2 or 3) to all 6 DoFs, and then normalize them back to [-1, 1] range.
    // This ensures that if one DoF is dominant, it will be preserved, while the others will be suppressed.
    float mag_6dof = 0.0f;
    for (int i = 0; i < 6; ++i) {
        mag_6dof += normalized_state[i] * normalized_state[i];
    }
    mag_6dof = sqrt(mag_6dof); // Magnitude of the 6DoF vector

    // Bail early if the magnitude is zero to avoid division by zero
    if (mag_6dof <= 1e-8f) {
        for (int i = 0; i < 12; ++i) {
            return_state[i] = 0.0f;
        }
        return;
    }

    // Handle power curve for isolation.
    // WARNING: Only values of 1, 2, 3, and 0.5 are optimized for RP2350 hardware. Other values will be slow.
    const float power = ISOLATION_POWER;

    float clamped_mag_6dof = (mag_6dof > 1.0f) ? 1.0f : mag_6dof; // Clamp to [0, 1] range
    float curved_mag_6dof = apply_power(clamped_mag_6dof, power);

    float snapped_mag = 0.0f;
    float snapped_effort[6] = {0.0f};

    for (int i = 0; i < 6; ++i) {
        float dir = normalized_state[i] / mag_6dof; // Direction of the DoF
        float sign = (dir >= 0.0f) ? 1.0f : -1.0f;

        snapped_effort[i] = sign * apply_power(fabsf(dir), power);
        snapped_mag += snapped_effort[i] * snapped_effort[i];
    }
    snapped_mag = sqrtf(snapped_mag); // Magnitude of the snapped effort vector

    // Reconstruct the gain for each DoF
    for (int i = 0; i < 6; ++i) {
        float final_pos = (snapped_effort[i] / snapped_mag) * curved_mag_6dof; // Scale by the curved magnitude

        // Determin the gain to apply to the velocity as well, based on the position gain.
        float axis_gain = (fabsf(normalized_state[i]) > 1e-8f) ? (final_pos / normalized_state[i]) : 0.0f;
        return_state[i] = final_pos; // Update the position
        return_state[i + 6] = normalized_state[i + 6] * axis_gain;
    }
}