#include "state_machine.h"

#if DEBUG_STATE_MACHINE_SERIAL
#define STATE_LOG_PRINT(...) Serial.print(__VA_ARGS__)
#define STATE_LOG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define STATE_LOG_PRINT(...)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           \
    do {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               \
    } while (0)
#define STATE_LOG_PRINTLN(...)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         \
    do {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               \
    } while (0)
#endif

/*
    GENERAL FUNCTIONS
*/
StateMachine::StateMachine(LEDController& led_controller, DipoleModel& dipole_model)
: m_led_controller(led_controller)
, m_dipole_model(dipole_model)
{
}

StateMachine::State StateMachine::get_state()
{
    return m_state;
}

String StateMachine::get_state_name()
{
    switch (m_state) {
        case State::BOOT:
            return "BOOT";
        case State::CHECK_SENSORS:
            return "CHECK_SENSORS";
        case State::SENSOR_ERROR:
            return "SENSOR_ERROR";
        case State::SENSOR_RECONNECT:
            return "SENSOR_RECONNECT";
        case State::CALIBRATION_FROM_FILE:
            return "CALIBRATION_FROM_FILE";
        case State::CALIBRATE_COLLECT:
            return "CALIBRATE_COLLECT";
        case State::CALIBRATE_COMPUTE:
            return "CALIBRATE_COMPUTE";
        case State::RUNNING:
            return "RUNNING";
        case State::RUNNING_WITHOUT_CALIBRATION:
            return "RUNNING_WITHOUT_CALIBRATION";
        case State::SLEEP:
            return "SLEEP";
        default:
            return "UNKNOWN_STATE";
    }
}

uint32_t StateMachine::get_last_state_change_time_ms()
{
    return m_last_state_change_time_ms;
}

void StateMachine::set_last_state_change_time_ms(uint32_t time_ms)
{
    m_last_state_change_time_ms = time_ms;
}

Calibration::LoadState StateMachine::get_calibration_load_state()
{
    return m_calibration_load_state;
}

uint32_t StateMachine::get_last_calibration_sample_time_ms()
{
    return m_last_calibration_sample_time_ms;
}

void StateMachine::set_last_calibration_sample_time_ms(uint32_t time_ms)
{
    m_last_calibration_sample_time_ms = time_ms;
}

uint32_t StateMachine::get_last_filtered_data_received_time_ms()
{
    return m_last_filtered_data_received_time_ms;
}

void StateMachine::set_last_filtered_data_received_time_ms(uint32_t time_ms)
{
    m_last_filtered_data_received_time_ms = time_ms;
}

/*
    ENTER STATE FUNCTIONS
*/

void StateMachine::enter_BOOT()
{
    change_state(State::BOOT);
    m_led_controller.set_solid(LED_BOOT_COLOR); // Solid YELLOW LED
    m_led_controller.on();
}

void StateMachine::enter_CHECK_SENSORS()
{
    change_state(State::CHECK_SENSORS);
}

void StateMachine::enter_SENSOR_ERROR()
{
    if (change_state(State::SENSOR_ERROR)) {
        m_led_controller.set_spinner(LED_ERROR_COLOR, 100, true, LEDController::SubMode::SPINNER_FULL); // Red spinner to indicate sensor error
    }
}

void StateMachine::enter_SENSOR_RECONNECT()
{
    change_state(State::SENSOR_RECONNECT);
}

void StateMachine::enter_CALIBRATION_FROM_FILE()
{
    if (change_state(State::CALIBRATION_FROM_FILE)) {
        m_led_controller.set_spinner(LED_CALIBRATION_COLOR, 100, true, LEDController::SubMode::SPINNER_FULL); // Blue spinner to indicate calibration
    }
}

void StateMachine::enter_CALIBRATE_COLLECT()
{
    State old_state = m_state;
    if (change_state(State::CALIBRATE_COLLECT)) {
        // Reset calibration data
        Calibration::reset_calibration_data();

        // Only set the spinner if we are not already coming from CALIBRATION_FROM_FILE
        if (old_state != State::CALIBRATION_FROM_FILE) {
            m_led_controller.set_spinner(LED_CALIBRATION_COLOR, 100, true, LEDController::SubMode::SPINNER_FULL); // Blue spinner to indicate calibration
        }
    }
}

void StateMachine::enter_CALIBRATE_COMPUTE()
{
    change_state(State::CALIBRATE_COMPUTE);
}

void StateMachine::enter_RUNNING()
{
    const State old_state = m_state;
    if (change_state(State::RUNNING)) {
        if (old_state == State::SLEEP) {
            m_led_controller.fade_on(LED_RUNNING_COLOR, 1000);
        }
        else {
            m_led_controller.set_solid(LED_RUNNING_COLOR); // Solid LEDs
            m_led_controller.on();
        }

        m_last_filtered_data_received_time_ms = millis(); // Reset the timeout timer for filtered data reception
    }
}

void StateMachine::enter_RUNNING_WITHOUT_CALIBRATION()
{
    if (change_state(State::RUNNING_WITHOUT_CALIBRATION)) {
        m_led_controller.set_solid(LED_RUNNING_WITHOUT_CALIBRATION_COLOR); // Solid ORANGE LED
        m_led_controller.on();

        m_last_filtered_data_received_time_ms = millis(); // Reset the timeout timer for filtered data reception
    }
}

void StateMachine::enter_SLEEP()
{
    const State old_state = m_state;
    if (change_state(State::SLEEP)) {
        if (old_state == State::RUNNING) {
            m_led_controller.fade_off(2000);
        }
        else {
            m_led_controller.off(); // Turn off LED
        }

        m_last_filtered_data_received_time_ms = millis(); // Reset the timeout timer for filtered data reception
    }
}

/*
    HANDLE STATE TRANSITIONS
*/

void StateMachine::handle_BOOT(uint32_t now)
{
    // After a short delay, transition to CHECK_SENSORS state
    if (now - m_last_state_change_time_ms > BOOT_DELAY_MS) {
        enter_CHECK_SENSORS();
    }
}

void StateMachine::handle_CHECK_SENSORS(bool sensor_status)
{
    if (sensor_status) {
        m_led_controller.queue_blinking_animation(LED_SUCCESS_COLOR, 200, 200, 1); // single blink GREEN -> successful sensor check
        enter_CALIBRATION_FROM_FILE();
    }
    else {
        enter_SENSOR_ERROR(); // Handles LED
    }
}

void StateMachine::handle_CALIBRATION_FROM_FILE()
{
    // Attempt to load calibration data from file
    CalibrationData loaded_data;
    if (Calibration::load_calibration_data(loaded_data)) {
        // Successfully loaded calibration data from file, apply it and proceed to RUNNING state
        m_dipole_model.set_magnetic_moments(loaded_data.magnetic_moments);
        m_dipole_model.set_offsets(loaded_data.offsets);
        m_calibration_load_state = Calibration::LoadState::FROM_FILE;

        m_led_controller.queue_blinking_animation(LED_CALIBRATION_SUCCESS_COLOR, 200, 200, 1); // blink CYAN one time to indicate successful calibration load
        enter_RUNNING();
    }
    else {
        // Failed to load calibration data from file, proceed to fresh calibration
        m_led_controller.queue_blinking_animation(LED_CALIBRATION_FAILURE_COLOR, 200, 200, 1); // blink MAGENTA one time to indicate failed calibration load

        enter_CALIBRATE_COLLECT();
    }
}

bool StateMachine::handle_CALIBRATE_COLLECT_partial(uint32_t now)
{
    // State to collect raw sensor data for calibration
    // Here, we only check completeness and timeout, the actual data collection is handled in main.cpp

    // Check completeness of calibration data collection
    if (Calibration::get_sample_count() >= CALIBRATION_SAMPLE_COUNT) {
        enter_CALIBRATE_COMPUTE();
        return true; // Indicate that we have transitioned to the next state
    }

    // Check for timeout, no new samples during 10 * CALIBRATION_SAMPLE_DELAY_MS
    if (now - m_last_state_change_time_ms > CALIBRATION_SAMPLE_TIMEOUT_MS) {
        m_led_controller.queue_blinking_animation(LED_CALIBRATION_FAILURE_COLOR, 200, 200, 2); // blink MAGENTA two times to indicate calibration collection timeout
        enter_SENSOR_RECONNECT();
        return true; // Indicate that we have transitioned to the next state
    }

    return false; // Indicate that we have not transitioned to the next state
}

void StateMachine::handle_CALIBRATE_COMPUTE()
{
    // Attempt fresh calibration
    float magnetic_moments[3];
    float offsets[6];
    bool calibration_success = Calibration::compute_calibration(magnetic_moments, offsets, m_dipole_model);

    if (calibration_success) {
        STATE_LOG_PRINTLN("Calibration successful, applying new calibration and saving to file.");

        // Calibration successful, apply the new calibration to the DipoleModel
        m_dipole_model.set_magnetic_moments(magnetic_moments);
        m_dipole_model.set_offsets(offsets);

        // Save the new calibration data to file
        CalibrationData new_data;
        m_dipole_model.get_magnetic_moments(new_data.magnetic_moments);
        m_dipole_model.get_offsets(new_data.offsets);

        if (Calibration::save_calibration_data(new_data)) {
            m_led_controller.queue_blinking_animation(LED_SUCCESS_COLOR, 200, 200, 1); // blink GREEN one time to indicate successful calibration & save
            STATE_LOG_PRINTLN("Calibration data saved successfully.");
            m_calibration_load_state = Calibration::LoadState::CALIBRATION_SUCCESSFUL_SAVED;
            enter_RUNNING();
        }
        else {
            // Failed to save calibration data to file, but calibration was successful
            m_led_controller.queue_blinking_animation(LED_CALIBRATION_FAILURE_COLOR, 200, 200, 4); // blink MAGENTA four times to indicate calibration success but save failure
            STATE_LOG_PRINTLN("Calibration successful, but failed to save calibration data to file.");
            m_calibration_load_state = Calibration::LoadState::CALIBRATION_SUCCESSFUL_SAVED_FAILED;
            enter_RUNNING();
        }
    }
    else {
        // Calibration failed, indicate failure and proceed to RUNNING_WITHOUT_CALIBRATION state
        STATE_LOG_PRINTLN("Calibration failed, checking calibration load state.");
        if (get_calibration_load_state() == Calibration::LoadState::NO_FILE_USING_DEFAULTS) {
            // Never had any calibration data, so we are running without calibration
            m_calibration_load_state = Calibration::LoadState::CALIBRATION_FAILED;
            enter_RUNNING_WITHOUT_CALIBRATION();
        } // No change in any other case, as we have either an old calibration from file or a previous successful calibration in memory
        m_led_controller.queue_blinking_animation(LED_CALIBRATION_FAILURE_COLOR, 200, 200, 3); // blink MAGENTA three times to indicate calibration failure
        enter_RUNNING();
    }
}

/*
    PRIVATE FUNCTIONS
*/

bool StateMachine::change_state(State new_state)
{
    if (new_state != m_state) {
        STATE_LOG_PRINT("== State change: ");
        STATE_LOG_PRINT(get_state_name());
        STATE_LOG_PRINT(" -> ");

        m_state = new_state;

        STATE_LOG_PRINTLN(get_state_name());

        m_last_state_change_time_ms = millis();

        return true; // State changed
    }
    return false; // No change
}