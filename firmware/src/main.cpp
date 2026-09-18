#include <Arduino.h>

#include "config.h"

#include "button_controller.h"
#include "calibration.h"
#include "dipole_model.h"
#include "extended_kalman_filter.h"
#include "hall_controller.h"
#include "hid_controller.h"
#include "led_controller.h"
#include "normalization.h"
#include "performance_profiler.h"
#include "state_machine.h"

#if DEBUG_MAIN_SERIAL
#include "helpers.h"
#endif

#if DEBUG_MAIN_SERIAL
#define MAIN_LOG_PRINT(...) Serial.print(__VA_ARGS__)
#define MAIN_LOG_PRINTLN(...) Serial.println(__VA_ARGS__)
#define MAIN_LOG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define MAIN_LOG_PRINT(...)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            \
    do {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               \
    } while (0)
#define MAIN_LOG_PRINTLN(...)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          \
    do {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               \
    } while (0)
#define MAIN_LOG_PRINTF(...)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           \
    do {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               \
    } while (0)
#endif

#if DEBUG_MAIN_PRINT_CORE1_DURATION || DEBUG_MAIN_SERIAL
#define MAIN_TIMING_LOG_PRINT(...) Serial.print(__VA_ARGS__)
#define MAIN_TIMING_LOG_PRINTLN(...) Serial.println(__VA_ARGS__)
#define MAIN_TIMING_LOG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define MAIN_TIMING_LOG_PRINT(...)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     \
    do {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               \
    } while (0)
#define MAIN_TIMING_LOG_PRINTLN(...)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   \
    do {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               \
    } while (0)
#define MAIN_TIMING_LOG_PRINTF(...)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    \
    do {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               \
    } while (0)
#endif

/*
    GLOBAL OBJECTS
*/

#ifdef BOARD_RP2350
// For RP2350, use the alternate I2C pins (SDA1, SCL1) for Wire1
HallSensorController hallController = HallSensorController(Wire1, PIN_MAG1_LS, PIN_MAG2_LS, PIN_MAG3_LS);
#else
// For RP2040, use the default I2C pins (SDA, SCL) for Wire
HallSensorController hallController = HallSensorController(Wire, PIN_MAG1_LS, PIN_MAG2_LS, PIN_MAG3_LS);
#endif
LEDController ledController = LEDController(PIN_LED_LS, PIN_LED_DATA, LED_COUNT);
DipoleModel dipoleModel = DipoleModel();
ExtendedKalmanFilter ekf = ExtendedKalmanFilter();
ButtonController buttonController = ButtonController(PIN_LEFT_BTN, PIN_RIGHT_BTN);
StateMachine stateMachine = StateMachine(ledController, dipoleModel);
HIDController hidController = HIDController();

/*
    SHARED DATA STRUCTURES FOR CORE 0 AND CORE 1 COMMUNICATION
*/

// Seqlock counters for lock-free single-writer/single-reader mailboxes.
// Even value: stable payload, odd value: writer is updating payload.
volatile uint32_t raw_mailbox_seq = 0;
volatile uint32_t filtered_mailbox_seq = 0;

// Shared raw sensor data mailbox (Core 0 producer -> Core 1 consumer)
struct RawSensorData {
    float rawData[9];      // 3 sensors, each with 3 axes (X, Y, Z)
    uint32_t timestamp_us; // Timestamp of the last update in microseconds
};
volatile RawSensorData sharedRawSensorData;

// Shared filtered data mailbox (Core 1 producer -> Core 0 consumer)
struct FilteredData {
    float x, y, z;       // Filtered translation data
    float rx, ry, rz;    // Filtered rotation data
    float vx, vy, vz;    // Filtered translation velocity data
    float vrx, vry, vrz; // Filtered rotation velocity data
    float dt;            // Time delta for the last update in seconds
};
volatile FilteredData sharedFilteredData;

/*
    THREAD-SAFE LAST FILTERED DATA STORAGE
    This is used to store the latest filtered data received from Core 1 in a thread-safe manner,
    so that it can be accessed in the main loop without race conditions.
*/

float latest_estimated_state[12] = {0.0f};

#if defined(ENABLE_PERFORMANCE_PROFILING) && (PERFORMANCE_PROFILING_LEVEL == 1)
namespace {
    uint32_t g_lite_last_filtered_arrival_us = 0;
    uint64_t g_lite_sum_interval_us = 0;
    uint32_t g_lite_interval_count = 0;
    uint32_t g_lite_min_interval_us = 0;
    uint32_t g_lite_max_interval_us = 0;
    uint32_t g_lite_last_print_ms = 0;

    void lite_profiler_on_new_filtered_value(uint32_t now_ms)
    {
        const uint32_t now_us = micros();

        if (g_lite_last_filtered_arrival_us != 0) {
            const uint32_t dt_us = now_us - g_lite_last_filtered_arrival_us;
            g_lite_sum_interval_us += dt_us;
            g_lite_interval_count += 1;

            if (g_lite_min_interval_us == 0 || dt_us < g_lite_min_interval_us) {
                g_lite_min_interval_us = dt_us;
            }
            if (dt_us > g_lite_max_interval_us) {
                g_lite_max_interval_us = dt_us;
            }
        }
        g_lite_last_filtered_arrival_us = now_us;

        if (g_lite_last_print_ms == 0) {
            g_lite_last_print_ms = now_ms;
            return;
        }

        if ((now_ms - g_lite_last_print_ms) < PERFORMANCE_PRINT_INTERVAL_MS) {
            return;
        }

        g_lite_last_print_ms = now_ms;
        if (g_lite_interval_count == 0) {
            return;
        }

        const float avg_interval_us = static_cast<float>(g_lite_sum_interval_us) / static_cast<float>(g_lite_interval_count);
        const float avg_hz = 1000000.0f / avg_interval_us;

        Serial.println("[PERFORMANCE][light]");
        Serial.print("  filtered_interval_avg_us: ");
        Serial.printf("%.2f", avg_interval_us);
        Serial.println();
        Serial.print("  filtered_rate_avg_hz: ");
        Serial.printf("%.2f", avg_hz);
        Serial.println();
        Serial.print("  filtered_interval_min_us: ");
        Serial.print(g_lite_min_interval_us);
        Serial.println();
        Serial.print("  filtered_interval_max_us: ");
        Serial.print(g_lite_max_interval_us);
        Serial.println();

        g_lite_sum_interval_us = 0;
        g_lite_interval_count = 0;
        g_lite_min_interval_us = 0;
        g_lite_max_interval_us = 0;
    }
}
#endif

void setup()
{
    Serial.begin(115200);

    // Initialize HID controller for USB communication
    hidController.begin();

    // Initialize the hall sensor controller
    hallController.begin();

    // Initialize the LED controller
    ledController.begin();

    // Initialize the button controller
    buttonController.begin();

    // Initialize the state machine
    stateMachine.enter_BOOT();

    // Initialize filesystem and load calibration data if available
    if (!Calibration::initialize_filesystem()) {
        ledController.queue_blinking_animation(LED_CALIBRATION_FAILURE_COLOR, 200, 200, 1); // blink MAGENTA one time to indicate filesystem initialization failure
    }
}

void loop()
{
    const uint32_t now = millis();
    float rawSensorData[9];
    uint8_t sensor_status;

    StateMachine::State current_state = stateMachine.get_state();

    switch (current_state) {
        case StateMachine::State::BOOT: {
            // Give some time for the system to stabilize, then transition to CHECK_SENSORS

            // Read raw sensor data from the hall sensors, do nothing with it just yet
            sensor_status = hallController.read(rawSensorData);

            stateMachine.handle_BOOT(now);
            break;
        }

        case StateMachine::State::CHECK_SENSORS: {
            // Check if the hall sensors are delivering valid data
            sensor_status = hallController.read(rawSensorData);
#if DEBUG_MAIN_SERIAL
            Helpers::print_raw_sensor_data(rawSensorData);
#endif

            stateMachine.handle_CHECK_SENSORS(sensor_status == HALL_STATUS_OK);
            break;
        }

        case StateMachine::State::SENSOR_ERROR: {
            // Failed to get hall sensor values, reattempt CHECK_SENSORS repeatedly after a delay
            if (now - stateMachine.get_last_state_change_time_ms() > SENSOR_RECONNECT_DELAY_MS) {
                stateMachine.enter_SENSOR_RECONNECT();
            }
            break;
        }

        case StateMachine::State::SENSOR_RECONNECT: {
            // Reattempt to reconnect to hall sensors
            hallController.begin(); // Reinitialize the hall sensor controller
            stateMachine.enter_CHECK_SENSORS();
            break;
        }

        case StateMachine::State::CALIBRATION_FROM_FILE: {
            // Attempt to load calibration data from file
            stateMachine.handle_CALIBRATION_FROM_FILE();
            break;
        }

        case StateMachine::State::CALIBRATE_COLLECT: {
            // Collect raw sensor data for calibration

            // Handle completeness and timeout checks for calibration collection
            if (stateMachine.handle_CALIBRATE_COLLECT_partial(now)) {
                break;
            }

            // Read raw sensor data from the hall sensors and add it to the calibration samples
            if (now - stateMachine.get_last_calibration_sample_time_ms() >= CALIBRATION_SAMPLE_DELAY_MS) {
                sensor_status = hallController.read(rawSensorData);
                if (sensor_status == HALL_STATUS_OK) {
                    Calibration::add_sample(rawSensorData);
                    stateMachine.set_last_calibration_sample_time_ms(now);
                }
            }
            break;
        }

        case StateMachine::State::CALIBRATE_COMPUTE: {
            stateMachine.handle_CALIBRATE_COMPUTE();
            break;
        }

        case StateMachine::State::RUNNING_NO_LED:
        case StateMachine::State::RUNNING_WITHOUT_CALIBRATION:
        case StateMachine::State::RUNNING: {
            // RUNNING state: Normal operation,
            // Check for Core 1 response and update the latest estimated state
            // read raw sensor data, send to Core 1 for processing
            // TODO: Send via HID to host computer

            // Consume latest filtered data using seqlock snapshot.
            static uint32_t last_filtered_seq = 0;
            uint32_t filtered_s1 = 0;
            uint32_t filtered_s2 = 0;
            FilteredData local_filtered = {};

            do {
                filtered_s1 = filtered_mailbox_seq;
                if (filtered_s1 & 1u) {
                    continue;
                }

                __dmb();
                local_filtered.x = sharedFilteredData.x;
                local_filtered.y = sharedFilteredData.y;
                local_filtered.z = sharedFilteredData.z;
                local_filtered.rx = sharedFilteredData.rx;
                local_filtered.ry = sharedFilteredData.ry;
                local_filtered.rz = sharedFilteredData.rz;
                local_filtered.vx = sharedFilteredData.vx;
                local_filtered.vy = sharedFilteredData.vy;
                local_filtered.vz = sharedFilteredData.vz;
                local_filtered.vrx = sharedFilteredData.vrx;
                local_filtered.vry = sharedFilteredData.vry;
                local_filtered.vrz = sharedFilteredData.vrz;
                local_filtered.dt = sharedFilteredData.dt;
                __dmb();

                filtered_s2 = filtered_mailbox_seq;
            } while ((filtered_s1 != filtered_s2) || (filtered_s2 & 1u));

            if (filtered_s2 != 0 && filtered_s2 != last_filtered_seq) {
                last_filtered_seq = filtered_s2;

                stateMachine.set_last_filtered_data_received_time_ms(now);

#if defined(ENABLE_PERFORMANCE_PROFILING) && (PERFORMANCE_PROFILING_LEVEL == 1)
                lite_profiler_on_new_filtered_value(now);
#endif

                latest_estimated_state[0] = local_filtered.x;
                latest_estimated_state[1] = local_filtered.y;
                latest_estimated_state[2] = local_filtered.z;
                latest_estimated_state[3] = local_filtered.rx;
                latest_estimated_state[4] = local_filtered.ry;
                latest_estimated_state[5] = local_filtered.rz;
                latest_estimated_state[6] = local_filtered.vx;
                latest_estimated_state[7] = local_filtered.vy;
                latest_estimated_state[8] = local_filtered.vz;
                latest_estimated_state[9] = local_filtered.vrx;
                latest_estimated_state[10] = local_filtered.vry;
                latest_estimated_state[11] = local_filtered.vrz;

                // Print roundtrip time between consecutive readings->filtering->return
                // Implies frequency of HID updates must be less than this
                float dt_ms = local_filtered.dt * 1e3;
                MAIN_TIMING_LOG_PRINT("Filter DT: ");
                MAIN_TIMING_LOG_PRINTF("%.3f", dt_ms);
                MAIN_TIMING_LOG_PRINTLN(" ms");

#if DEBUG_MAIN_SERIAL
                // Print the latest estimated state for debugging
                Helpers::print_estimated_state(latest_estimated_state);
                // Condensed print for debugging
                // Helpers::print_condensed_estimated_state(latest_estimated_state);
#endif
            }

            // Publish latest raw sample (overwrite mailbox; no queue backlog).
            PERFORMANCE_BEGIN(0, PerformanceProfiler::Section::CORE0_SENSOR_READ);
            sensor_status = hallController.read(rawSensorData);
            PERFORMANCE_END(0, PerformanceProfiler::Section::CORE0_SENSOR_READ);

#if DEBUG_MAIN_SERIAL
            Helpers::print_raw_sensor_data(rawSensorData);
#endif

            if (sensor_status == HALL_STATUS_OK) {
                PERFORMANCE_BEGIN(0, PerformanceProfiler::Section::CORE0_HANDOVER);
                const uint32_t seq0 = raw_mailbox_seq;
                raw_mailbox_seq = seq0 + 1u; // mark write-in-progress (odd)
                __dmb();

                for (int i = 0; i < 9; ++i) {
                    sharedRawSensorData.rawData[i] = rawSensorData[i];
                }
                sharedRawSensorData.timestamp_us = micros();

                __dmb();
                raw_mailbox_seq = seq0 + 2u; // mark stable payload (even)
                PERFORMANCE_END(0, PerformanceProfiler::Section::CORE0_HANDOVER);
            }

            // TODO: Send HID data at fixed intervals > dt of Kalman

            // Timeout check (no sensor readings) -> SENSOR_RECONNECT state
            if (now - stateMachine.get_last_filtered_data_received_time_ms() > RUNNING_STATE_READ_ERROR_TIMEOUT_MS) {
                stateMachine.enter_SENSOR_ERROR();
                break;
            }

            // Transition between different RUNNING_... states
            if (now - hidController.get_last_report_time_ms() > RUNNING_STATE_INACTIVITY_TIMEOUT_MS) {
                // Transition to RUNNING_NO_LED on inactivity.
                // We can infer inactivity by checking time last HID report was sent
                // as we only send HID on changes in axes or buttons.
                stateMachine.enter_RUNNING_NO_LED(); // does nothing if already in RUNNING_NO_LED state
                break;
            }
            else {
                // Transition back to RUNNING or RUNNING_WITHOUT_CALIBRATION on activity,
                // depending on whether calibration data is available.
                if (stateMachine.get_calibration_load_state() != Calibration::LoadState::NO_FILE_USING_DEFAULTS) {
                    stateMachine.enter_RUNNING(); // does nothing if already in RUNNING state
                    break;
                }
                else {
                    stateMachine.enter_RUNNING_WITHOUT_CALIBRATION(); // does nothing if already in RUNNING_WITHOUT_CALIBRATION state
                    break;
                }
            }
            break;
        }
        default: {
            // Serial.println("Unknown state. Should not happen.");
            break;
        }
    }

    // Safe to access latest_estimated_state here for any other processing or output
    // For example, you could use it to update a display, send over serial, etc

    // Updates
    buttonController.update(); // Do first, so we can react to button presses immediately

    // Handle combo button states first
    ButtonController::ComboState combo_state = buttonController.getComboState();
    if (combo_state == ButtonController::ComboState::LONG_PRESSED) {
        stateMachine.enter_CALIBRATE_COLLECT();
    }

    static uint16_t buttons = 0; // Initialize buttons to 0 (no buttons pressed)

    ButtonController::ButtonState left_button_state = buttonController.getLeftButtonState();
    ButtonController::ButtonState right_button_state = buttonController.getRightButtonState();

    if (left_button_state == ButtonController::ButtonState::PRESSED) {
        MAIN_LOG_PRINTLN("Left button pressed");
        buttons |= 0x0001; // Set bit 0 for left button press
    }
    else if (left_button_state == ButtonController::ButtonState::RELEASED) {
        MAIN_LOG_PRINTLN("Left button released");
        buttons &= ~0x0001; // Clear bit 0 for left button release
    }
    if (right_button_state == ButtonController::ButtonState::PRESSED) {
        MAIN_LOG_PRINTLN("Right button pressed");
        buttons |= 0x0002; // Set bit 1 for right button press
    }
    else if (right_button_state == ButtonController::ButtonState::RELEASED) {
        MAIN_LOG_PRINTLN("Right button released");
        buttons &= ~0x0002; // Clear bit 1 for right button release
    }
    hidController.sendReport(latest_estimated_state, buttons);

    // LED controller update
    ledController.update(
      latest_estimated_state[0],
      latest_estimated_state[1],
      latest_estimated_state[2],
      latest_estimated_state[3],
      latest_estimated_state[4],
      latest_estimated_state[5]);

#if defined(ENABLE_PERFORMANCE_PROFILING) && (PERFORMANCE_PROFILING_LEVEL >= 2)
    PerformanceProfiler::print_if_due(0, now, PERFORMANCE_PRINT_INTERVAL_MS);
#endif
}

void setup1()
{
    const float initial_state[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // Initial pose: x, y, z, rx, ry, rz
    ekf.init(initial_state, EKF_PROCESS_NOISE_STD, EKF_SENSOR_NOISE_STD);
}

void loop1()
{
    // Consume latest raw sample using seqlock snapshot.
    static uint32_t last_time_us = 0;
    static uint32_t last_raw_seq = 0;
    static bool is_first_run = true;

    uint32_t raw_s1 = 0;
    uint32_t raw_s2 = 0;
    RawSensorData local_sample = {};

    do {
        raw_s1 = raw_mailbox_seq;
        if (raw_s1 & 1u) {
            continue;
        }

        __dmb();
        for (int i = 0; i < 9; ++i) {
            local_sample.rawData[i] = sharedRawSensorData.rawData[i];
        }
        local_sample.timestamp_us = sharedRawSensorData.timestamp_us;
        __dmb();

        raw_s2 = raw_mailbox_seq;
    } while ((raw_s1 != raw_s2) || (raw_s2 & 1u));

    if (raw_s2 == 0 || raw_s2 == last_raw_seq) {
        return;
    }
    last_raw_seq = raw_s2;

    // On first run, we don't have a previous timestamp to calculate dt,
    // but we still can add data to the EKF and establish a baseline.
    if (is_first_run) {
        // Store timestamp of first data
        last_time_us = local_sample.timestamp_us;
        is_first_run = false;

        // Update EKF with the first set of raw sensor data to establish a baseline
        float local_raw[9];
        for (int i = 0; i < 9; ++i) {
            local_raw[i] = local_sample.rawData[i];
        }
        ekf.update(local_raw, dipoleModel);

        // Publish filtered baseline once so Core 0 can consume it.
        const uint32_t filtered_seq0 = filtered_mailbox_seq;
        filtered_mailbox_seq = filtered_seq0 + 1u;
        __dmb();

        float estimated_state_first[12];
        float deadzone_normalized_state_first[12];
        ekf.get_state(estimated_state_first);
        Normalization::apply_normalization_deadzone_isolation(estimated_state_first, deadzone_normalized_state_first);

        sharedFilteredData.x = deadzone_normalized_state_first[0];
        sharedFilteredData.y = deadzone_normalized_state_first[1];
        sharedFilteredData.z = deadzone_normalized_state_first[2];
        sharedFilteredData.rx = deadzone_normalized_state_first[3];
        sharedFilteredData.ry = deadzone_normalized_state_first[4];
        sharedFilteredData.rz = deadzone_normalized_state_first[5];
        sharedFilteredData.vx = deadzone_normalized_state_first[6];
        sharedFilteredData.vy = deadzone_normalized_state_first[7];
        sharedFilteredData.vz = deadzone_normalized_state_first[8];
        sharedFilteredData.vrx = deadzone_normalized_state_first[9];
        sharedFilteredData.vry = deadzone_normalized_state_first[10];
        sharedFilteredData.vrz = deadzone_normalized_state_first[11];
        sharedFilteredData.dt = 0.0f;

        __dmb();
        filtered_mailbox_seq = filtered_seq0 + 2u;
        return;
    }

    // Calculate dt based on the timestamp of the latest raw sensor data
    float dt = (local_sample.timestamp_us - last_time_us) * 1e-6f; // Convert microseconds to seconds
    last_time_us = local_sample.timestamp_us;

    // Guard against massive timing spikes or clock hiccups
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.001f; // Fallback to a default 1ms step if timing fails
    }

    // Local copy to isolate processing memory
    PERFORMANCE_BEGIN(1, PerformanceProfiler::Section::CORE1_TOTAL);

    float local_raw[9];
    for (int i = 0; i < 9; ++i) {
        local_raw[i] = local_sample.rawData[i];
    }

    // Step the Kalman Filter math engine forward
    PERFORMANCE_BEGIN(1, PerformanceProfiler::Section::CORE1_PREDICT);
    ekf.predict(dt);
    PERFORMANCE_END(1, PerformanceProfiler::Section::CORE1_PREDICT);

    PERFORMANCE_BEGIN(1, PerformanceProfiler::Section::CORE1_UPDATE);
    ekf.update(local_raw, dipoleModel);
    PERFORMANCE_END(1, PerformanceProfiler::Section::CORE1_UPDATE);

    // Extract the filtered pose from the EKF state vector
    float estimated_state[12];
    ekf.get_state(estimated_state);

    // Apply normalization, deadzone, and curved isolation
    // This
    // - normalizes trans and rot to [-1, 1] and applies same factor to velocities
    // - applies deadzone to trans vector magnitude and rot vector magnitude
    // - applies curved isolation over all 6 DoF combined (power law + renormalization) and applies same factor to velocities
    float deadzone_normalized_state[12];
    PERFORMANCE_BEGIN(1, PerformanceProfiler::Section::CORE1_NORMALIZATION);
    Normalization::apply_normalization_deadzone_isolation(estimated_state, deadzone_normalized_state);
    PERFORMANCE_END(1, PerformanceProfiler::Section::CORE1_NORMALIZATION);

    // Publish processed state mailbox for Core 0.
    const uint32_t filtered_seq0 = filtered_mailbox_seq;
    filtered_mailbox_seq = filtered_seq0 + 1u; // mark write-in-progress
    __dmb();

    sharedFilteredData.x = deadzone_normalized_state[0];
    sharedFilteredData.y = deadzone_normalized_state[1];
    sharedFilteredData.z = deadzone_normalized_state[2];
    sharedFilteredData.rx = deadzone_normalized_state[3];
    sharedFilteredData.ry = deadzone_normalized_state[4];
    sharedFilteredData.rz = deadzone_normalized_state[5];
    sharedFilteredData.vx = deadzone_normalized_state[6];
    sharedFilteredData.vy = deadzone_normalized_state[7];
    sharedFilteredData.vz = deadzone_normalized_state[8];
    sharedFilteredData.vrx = deadzone_normalized_state[9];
    sharedFilteredData.vry = deadzone_normalized_state[10];
    sharedFilteredData.vrz = deadzone_normalized_state[11];
    sharedFilteredData.dt = dt;

    __dmb();
    filtered_mailbox_seq = filtered_seq0 + 2u; // mark stable payload

    PERFORMANCE_END(1, PerformanceProfiler::Section::CORE1_TOTAL);

#if defined(ENABLE_PERFORMANCE_PROFILING) && (PERFORMANCE_PROFILING_LEVEL >= 2)
    PerformanceProfiler::print_if_due(1, millis(), PERFORMANCE_PRINT_INTERVAL_MS);
#endif
}