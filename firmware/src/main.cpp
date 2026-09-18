#include <Arduino.h>

#include "config.h"

#include "button_controller.h"
#include "calibration.h"
#include "core_mailbox.h"
#include "dipole_model.h"
#include "extended_kalman_filter.h"
#include "hall_controller.h"
#include "hid_controller.h"
#include "led_controller.h"
#include "normalization.h"
#include "performance_profiler.h"
#include "power_manager.h"
#include "sleep_controller.h"
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
SleepController sleepController = SleepController();

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
    PowerManager::begin();

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

        case StateMachine::State::RUNNING_WITHOUT_CALIBRATION:
        case StateMachine::State::RUNNING: {
            // RUNNING state: Normal operation
            // Check for Core 1 response and update the latest estimated state
            // read raw sensor data, send to Core 1 for processing

            // Ensure modules and Core 1 are in the correct state
            PowerManager::exitSleep();
            hallController.enterFastMode();
            CoreMailbox::setCore1Sleeping(false);

            static uint32_t last_filtered_seq = 0;
            CoreMailbox::FilteredData local_filtered = {};

            if (CoreMailbox::consumeFiltered(last_filtered_seq, local_filtered)) {

                stateMachine.set_last_filtered_data_received_time_ms(now);

#if defined(ENABLE_PERFORMANCE_PROFILING) && (PERFORMANCE_PROFILING_LEVEL == 1)
                lite_profiler_on_new_filtered_value(now);
#endif

                for (int index = 0; index < 12; ++index) {
                    latest_estimated_state[index] = local_filtered.state[index];
                }

                // Print roundtrip time between consecutive readings->filtering->return
                // Implies frequency of HID updates must be less than this
                float dt_ms = local_filtered.dt * 1e3;
                MAIN_TIMING_LOG_PRINT("Filter DT: ");
                MAIN_TIMING_LOG_PRINT(dt_ms);
                MAIN_TIMING_LOG_PRINTLN(" ms");

#if DEBUG_MAIN_SERIAL
                // Print the latest estimated state for debugging
                Helpers::print_estimated_state(latest_estimated_state);
                // Condensed print for debugging
                // Helpers::print_condensed_estimated_state(latest_estimated_state);
#endif
            }

            PERFORMANCE_BEGIN(0, PerformanceProfiler::Section::CORE0_SENSOR_READ);
            sensor_status = hallController.read(rawSensorData);
            PERFORMANCE_END(0, PerformanceProfiler::Section::CORE0_SENSOR_READ);
#if DEBUG_MAIN_SERIAL
            Helpers::print_raw_sensor_data(rawSensorData);
#endif

            if (sensor_status == HALL_STATUS_OK) {
                PERFORMANCE_BEGIN(0, PerformanceProfiler::Section::CORE0_HANDOVER);
                CoreMailbox::publishRaw(rawSensorData, micros());
                PERFORMANCE_END(0, PerformanceProfiler::Section::CORE0_HANDOVER);
            }

            if (now - stateMachine.get_last_filtered_data_received_time_ms() > RUNNING_STATE_READ_ERROR_TIMEOUT_MS) {
                stateMachine.enter_SENSOR_ERROR();
                break;
            }

            const bool wake_grace_active = sleepController.wakeGraceActive(now);
            if (!wake_grace_active && now - hidController.get_last_report_time_ms() > RUNNING_STATE_INACTIVITY_TIMEOUT_MS) {
                // Transition to SLEEP on inactivity.
                // We can infer inactivity by checking time last HID report was sent
                // as we only send HID on changes in axes or buttons.
                stateMachine.enter_SLEEP();
            }
            else {
                if (stateMachine.get_calibration_load_state() != Calibration::LoadState::NO_FILE_USING_DEFAULTS) {
                    stateMachine.enter_RUNNING(); // does nothing if already in RUNNING state
                }
                else {
                    stateMachine.enter_RUNNING_WITHOUT_CALIBRATION(); // does nothing if already in RUNNING_WITHOUT_CALIBRATION state
                }
            }
            break;
        }

        case StateMachine::State::SLEEP: {
            CoreMailbox::setCore1Sleeping(true);
            const bool host_activity_recent = sleepController.wakeGraceActive(now) || now - hidController.get_last_report_time_ms() <= RUNNING_STATE_INACTIVITY_TIMEOUT_MS;
            const SleepController::Result sleep_result = sleepController.update(now, host_activity_recent, hallController, ledController);

            if (sleep_result == SleepController::Result::SENSOR_TIMEOUT) {
                stateMachine.enter_SENSOR_ERROR();
                break;
            }

            if (sleep_result == SleepController::Result::WAKE_REQUESTED) {
                CoreMailbox::setCore1Sleeping(false);
                for (int i = 0; i < 12; i++) {
                    latest_estimated_state[i] = 0.0;
                }
                hidController.sendReport(latest_estimated_state, 0, true);

                if (stateMachine.get_calibration_load_state() != Calibration::LoadState::NO_FILE_USING_DEFAULTS) {
                    stateMachine.enter_RUNNING(); // does nothing if already in RUNNING state
                }
                else {
                    stateMachine.enter_RUNNING_WITHOUT_CALIBRATION(); // does nothing if already in RUNNING_WITHOUT_CALIBRATION state
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
    hidController.sendReport(latest_estimated_state, buttons, false);

    // LED controller update
    ledController.update(latest_estimated_state[0], latest_estimated_state[1], latest_estimated_state[3], latest_estimated_state[4]);

#if defined(ENABLE_PERFORMANCE_PROFILING) && (PERFORMANCE_PROFILING_LEVEL >= 2)
    PerformanceProfiler::print_if_due(0, now, PERFORMANCE_PRINT_INTERVAL_MS);
#endif

    // Delay next iteration if sleeping
    if (PowerManager::sleepActive()) {
        delay(SLEEP_SAMPLE_INTERVAL_MS);
    }
}

void setup1()
{
    const float initial_state[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // Initial pose: x, y, z, rx, ry, rz
    ekf.init(initial_state, EKF_PROCESS_NOISE_STD, EKF_SENSOR_NOISE_STD);
}

void loop1()
{
    if (CoreMailbox::core1Sleeping()) {
        delay(SLEEP_SAMPLE_INTERVAL_MS);
        return;
    }

    static uint32_t last_time_us = 0;
    static uint32_t last_raw_seq = 0;
    static bool is_first_run = true;
    CoreMailbox::RawSensorData local_sample = {};

    if (!CoreMailbox::consumeRaw(last_raw_seq, local_sample)) {
        return;
    }

    // On first run, we don't have a previous timestamp to calculate dt,
    // but we still can add data to the EKF and establish a baseline.
    if (is_first_run) {
        // Store timestamp of first data
        last_time_us = local_sample.timestamp_us;
        is_first_run = false;

        // Update EKF with the first set of raw sensor data to establish a baseline
        float local_raw[9];
        for (int i = 0; i < 9; ++i) {
            local_raw[i] = local_sample.values[i];
        }
        ekf.update(local_raw, dipoleModel);

        float estimated_state_first[12];
        float deadzone_normalized_state_first[12];
        ekf.get_state(estimated_state_first);
        Normalization::apply_normalization_deadzone_isolation(estimated_state_first, deadzone_normalized_state_first);
        CoreMailbox::publishFiltered(deadzone_normalized_state_first, 0.0f);
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
        local_raw[i] = local_sample.values[i];
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

    CoreMailbox::publishFiltered(deadzone_normalized_state, dt);

    PERFORMANCE_END(1, PerformanceProfiler::Section::CORE1_TOTAL);

#if defined(ENABLE_PERFORMANCE_PROFILING) && (PERFORMANCE_PROFILING_LEVEL >= 2)
    PerformanceProfiler::print_if_due(1, millis(), PERFORMANCE_PRINT_INTERVAL_MS);
#endif
}