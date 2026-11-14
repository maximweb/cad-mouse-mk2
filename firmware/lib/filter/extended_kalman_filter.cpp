#include "extended_kalman_filter.h"
#include "performance_profiler.h"
#include <algorithm> // for std::isfinite
#include <cmath>
#include <cstring>

#if DEBUG_KALMAN_FILTER_SERIAL
#define EKF_LOG_PRINT(...) Serial.print(__VA_ARGS__)
#define EKF_LOG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define EKF_LOG_PRINT(...)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             \
    do {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               \
    } while (0)
#define EKF_LOG_PRINTLN(...)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           \
    do {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               \
    } while (0)
#endif

ExtendedKalmanFilter::ExtendedKalmanFilter() {}

void ExtendedKalmanFilter::init(const float initial_state[6], float process_noise_std, float sensor_noise_std)
{
    // Initialize state vector, strictly guarding against uninitialized NaNs
    for (int i = 0; i < 6; ++i) {
        m_x[i] = std::isfinite(initial_state[i]) ? initial_state[i] : 0.0f;
    }
    for (int i = 6; i < 12; ++i) {
        m_x[i] = 0.0f;
    }

    std::memset(m_P, 0, sizeof(m_P));
    for (int i = 0; i < 6; ++i)
        m_P[i][i] = 0.5f;
    for (int i = 6; i < 12; ++i)
        m_P[i][i] = 1.0f;

    m_R_var = sensor_noise_std * sensor_noise_std;

    m_Q_pos = 0.01f * process_noise_std;
    m_Q_rot = 0.01f * process_noise_std;
    m_Q_vel = 1.0f * process_noise_std;
    m_Q_ang_vel = 1.0f * process_noise_std;

    std::memset(m_cached_H, 0, sizeof(m_cached_H));
}

void ExtendedKalmanFilter::predict(float dt)
{
    // FIX: Cap dt to prevent massive integration jumps on first boot or thread stalls
    if (dt <= 0.0f)
        return;
    if (dt > 0.1f)
        dt = 0.1f;

    // FIX: Sanitize state before predicting to prevent garbage propagation
    for (int i = 0; i < 12; ++i) {
        if (!std::isfinite(m_x[i]))
            m_x[i] = 0.0f;
    }

    // 1. Predict State
    for (int i = 0; i < 6; ++i) {
        m_x[i] += m_x[i + 6] * dt;
    }

    // 2. Block-form F * P * F^T propagation for constant-velocity model.
    const float dt2 = dt * dt;
    float A[6][6];
    float B[6][6];
    float C[6][6];
    float D[6][6];

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            A[i][j] = m_P[i][j];
            B[i][j] = m_P[i][j + 6];
            C[i][j] = m_P[i + 6][j];
            D[i][j] = m_P[i + 6][j + 6];
        }
    }

    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            m_P[i][j] = A[i][j] + dt * (B[i][j] + C[i][j]) + dt2 * D[i][j];
            m_P[i][j + 6] = B[i][j] + dt * D[i][j];
            m_P[i + 6][j] = C[i][j] + dt * D[i][j];
            m_P[i + 6][j + 6] = D[i][j];
        }
    }

    // 3. Inject Process Noise Covariance (Q)
    for (int i = 0; i < 3; ++i)
        m_P[i][i] += m_Q_pos * dt;
    for (int i = 3; i < 6; ++i)
        m_P[i][i] += m_Q_rot * dt;
    for (int i = 6; i < 9; ++i)
        m_P[i][i] += m_Q_vel * dt;
    for (int i = 9; i < 12; ++i)
        m_P[i][i] += m_Q_ang_vel * dt;

    // FIX: Enforce Symmetry immediately after prediction
    for (int i = 0; i < 12; ++i) {
        for (int j = i + 1; j < 12; ++j) {
            float avg = (m_P[i][j] + m_P[j][i]) * 0.5f;
            m_P[i][j] = avg;
            m_P[j][i] = avg;
        }
    }
}

void ExtendedKalmanFilter::compute_jacobian(const float state[12], float H[9][12], DipoleModel& dipole_model, float base_readings_out[9])
{
    float local_base_B[9];
    float* base_B = (base_readings_out != nullptr) ? base_readings_out : local_base_B;
    std::memset(H, 0, sizeof(float) * 9 * 12);

#if EKF_JACOBIAN_MODE == 1
    float d_readings_d_xyz[9][3];
    dipole_model.get_expected_readings_with_translation_jacobian(state[0], state[1], state[2], state[3], state[4], state[5], base_B, d_readings_d_xyz);

    // Analytic translation columns.
    for (int i = 0; i < 9; ++i) {
        H[i][0] = d_readings_d_xyz[i][0];
        H[i][1] = d_readings_d_xyz[i][1];
        H[i][2] = d_readings_d_xyz[i][2];
    }

    // Numeric rotation columns.
    for (int j = 3; j < 6; ++j) {
        float perturbed_pose[6] = {state[0], state[1], state[2], state[3], state[4], state[5]};

        float state_val = perturbed_pose[j];
        if (!std::isfinite(state_val)) {
            state_val = 0.0f;
        }

        float h = 1e-4f * std::fabs(state_val);
        if (h < 1e-4f)
            h = 1e-4f;

        perturbed_pose[j] = state_val + h;

        float perturbed_B[9];
        dipole_model.get_expected_readings(perturbed_pose[0], perturbed_pose[1], perturbed_pose[2], perturbed_pose[3], perturbed_pose[4], perturbed_pose[5], perturbed_B);

        for (int i = 0; i < 9; ++i) {
            float diff = perturbed_B[i] - base_B[i];
            H[i][j] = std::isfinite(diff) ? (diff / h) : 0.0f;
        }
    }
#else
    dipole_model.get_expected_readings(state[0], state[1], state[2], state[3], state[4], state[5], base_B);

    for (int j = 0; j < 6; ++j) {
        float perturbed_pose[6] = {state[0], state[1], state[2], state[3], state[4], state[5]};

        float state_val = perturbed_pose[j];
        if (!std::isfinite(state_val)) {
            state_val = 0.0f;
        }

        float h = 1e-4f * std::fabs(state_val);
        if (h < 1e-4f)
            h = 1e-4f;

        perturbed_pose[j] = state_val + h;

        float perturbed_B[9];
        dipole_model.get_expected_readings(perturbed_pose[0], perturbed_pose[1], perturbed_pose[2], perturbed_pose[3], perturbed_pose[4], perturbed_pose[5], perturbed_B);

        for (int i = 0; i < 9; ++i) {
            float diff = perturbed_B[i] - base_B[i];
            H[i][j] = std::isfinite(diff) ? (diff / h) : 0.0f;
        }
    }
#endif
}

void ExtendedKalmanFilter::update(float sensor_readings[9], DipoleModel& dipole_model)
{
    // FIX: Reject bad I2C/ADC reads entirely instead of poisoning the filter
    for (int i = 0; i < 9; ++i) {
        if (!std::isfinite(sensor_readings[i]))
            return;
    }

    float h_x[9];
    float y[9];
    auto update_innovation = [&]() {
        for (int i = 0; i < 9; ++i) {
            const float diff = sensor_readings[i] - h_x[i];
            y[i] = std::isfinite(diff) ? diff : 0.0f;
        }
    };

    PERFORMANCE_BEGIN(1, PerformanceProfiler::Section::EKF_JACOBIAN);
    compute_jacobian(m_x, m_cached_H, dipole_model, h_x);
    PERFORMANCE_END(1, PerformanceProfiler::Section::EKF_JACOBIAN);

    PERFORMANCE_BEGIN(1, PerformanceProfiler::Section::EKF_MODEL_HX);
    PERFORMANCE_END(1, PerformanceProfiler::Section::EKF_MODEL_HX);
    update_innovation();

    float (*H)[12] = m_cached_H;

    EKF_LOG_PRINT("[DEBUG] Residual Y[0]: ");
    EKF_LOG_PRINT(y[0]);
    EKF_LOG_PRINT(" | h_x[0]: ");
    EKF_LOG_PRINTLN(h_x[0]);

    // Measurement model depends only on pose states (0..5), so H = [H_pose, 0].
    // Keep H*P in one contiguous block to avoid branchy pose/vel access in hot paths.
    float HP_full[9][12] = {0};
    PERFORMANCE_BEGIN(1, PerformanceProfiler::Section::EKF_HP);
    for (int i = 0; i < 9; ++i) {
        float sum_full[12] = {0};

        for (int k = 0; k < 6; ++k) {
            const float h_ik = H[i][k];
            for (int j = 0; j < 12; ++j) {
                sum_full[j] += h_ik * m_P[k][j];
            }
        }

        for (int j = 0; j < 12; ++j) {
            HP_full[i][j] = sum_full[j];
        }
    }
    PERFORMANCE_END(1, PerformanceProfiler::Section::EKF_HP);

    float S[9][9] = {0};
    PERFORMANCE_BEGIN(1, PerformanceProfiler::Section::EKF_S);
    for (int i = 0; i < 9; ++i) {
        for (int j = i; j < 9; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < 6; ++k) {
                sum += HP_full[i][k] * H[j][k];
            }
            if (i == j) {
                sum += m_R_var;
            }
            S[i][j] = sum;
            if (j != i) {
                S[j][i] = sum;
            }
        }
    }
    PERFORMANCE_END(1, PerformanceProfiler::Section::EKF_S);

    // --- 1. STABILIZED CHOLESKY DECOMPOSITION ---
    float L[9][9] = {0};
    const float eps = 1e-3f;

    PERFORMANCE_BEGIN(1, PerformanceProfiler::Section::EKF_CHOLESKY);
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j <= i; j++) {
            float sum = 0;
            for (int k = 0; k < j; k++)
                sum += L[i][k] * L[j][k];

            if (i == j) {
                float val = (S[i][i] + eps) - sum;
                if (val < 1e-5f)
                    val = 1e-5f;
                L[i][j] = std::sqrt(val);
            }
            else {
                L[i][j] = (S[i][j] - sum) / (L[j][j] < 1e-5f ? 1e-5f : L[j][j]);
            }
        }
    }
    PERFORMANCE_END(1, PerformanceProfiler::Section::EKF_CHOLESKY);

    EKF_LOG_PRINT("[DEBUG] Matrix S Diagonal [0]: ");
    EKF_LOG_PRINT(S[0][0]);
    EKF_LOG_PRINT(" | L Diagonal [0]: ");
    EKF_LOG_PRINTLN(L[0][0]);

    // --- 2. SOLVE S * iY = y ---
    PERFORMANCE_BEGIN(1, PerformanceProfiler::Section::EKF_SOLVE_IY);
    float iY_temp[9] = {0};
    for (int i = 0; i < 9; i++) {
        float sum = 0;
        for (int k = 0; k < i; k++)
            sum += L[i][k] * iY_temp[k];
        iY_temp[i] = (y[i] - sum) / L[i][i];
    }
    float iY[9] = {0};
    for (int i = 8; i >= 0; i--) {
        float sum = 0;
        for (int k = i + 1; k < 9; k++)
            sum += L[k][i] * iY[k];
        iY[i] = (iY_temp[i] - sum) / L[i][i];
    }
    PERFORMANCE_END(1, PerformanceProfiler::Section::EKF_SOLVE_IY);

    // --- 3. STATE UPDATE ---
    PERFORMANCE_BEGIN(1, PerformanceProfiler::Section::EKF_STATE_CORRECTION);
    for (int i = 0; i < 12; ++i) {
        float correction = 0.0f;
        for (int j = 0; j < 9; ++j) {
            correction += HP_full[j][i] * iY[j];
        }

        if (std::isfinite(correction)) {
            m_x[i] += correction;
        }
    }
    PERFORMANCE_END(1, PerformanceProfiler::Section::EKF_STATE_CORRECTION);

    // --- 4. UPDATE COVARIANCE MATRIX ---
    PERFORMANCE_BEGIN(1, PerformanceProfiler::Section::EKF_COVARIANCE_UPDATE);
    // With S = L*L^T and Y_tmp = (P*H^T)*L^{-T}, the covariance
    // correction is Y_tmp*Y_tmp^T. This avoids the backward solve for K.
    float Y_tmp[12][9];

    // Forward solve: Y_tmp * L^T = P*H^T  (row-wise RHS)
    for (int i = 0; i < 9; ++i) {
        const float lii = (L[i][i] < 1e-5f) ? 1e-5f : L[i][i];
        for (int row = 0; row < 12; ++row) {
            float sum = 0.0f;
            for (int k = 0; k < i; ++k) {
                sum += L[i][k] * Y_tmp[row][k];
            }
            Y_tmp[row][i] = (HP_full[i][row] - sum) / lii;
        }
    }

    for (int row = 0; row < 12; ++row) {
        // Update only upper triangle and mirror immediately to preserve symmetry.
        for (int col = row; col < 12; ++col) {
            const float delta = Y_tmp[row][0] * Y_tmp[col][0] + Y_tmp[row][1] * Y_tmp[col][1] + Y_tmp[row][2] * Y_tmp[col][2] + Y_tmp[row][3] * Y_tmp[col][3] + Y_tmp[row][4] * Y_tmp[col][4] + Y_tmp[row][5] * Y_tmp[col][5] + Y_tmp[row][6] * Y_tmp[col][6] + Y_tmp[row][7] * Y_tmp[col][7] + Y_tmp[row][8] * Y_tmp[col][8];

            float updated = m_P[row][col] - delta;
            if (!std::isfinite(updated)) {
                updated = m_P[row][col];
            }

            m_P[row][col] = updated;
            if (col != row) {
                m_P[col][row] = updated;
            }
        }
    }

    // Keep covariance numerically valid.
    for (int i = 0; i < 12; ++i) {
        if (m_P[i][i] < 1e-6f) {
            m_P[i][i] = 1e-6f;
        }
    }
    PERFORMANCE_END(1, PerformanceProfiler::Section::EKF_COVARIANCE_UPDATE);
}

void ExtendedKalmanFilter::get_state(float state_out[12])
{
    for (int i = 0; i < 12; ++i)
        state_out[i] = m_x[i];
}