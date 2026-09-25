#pragma once

#include <Arduino.h>

#include "calibration.h"
#include "dipole_model.h"
#include "led_controller.h"

class StateMachine {
public:
    enum class State {
        /*
            Initial BOOT state
            - Defined starting point for the state machine
            - Short delay to allow the system to stabilize, then transition to CHECK_SENSORS
            - Visualizer (permanent): Solid YELLOW LED
        */
        BOOT,
        /*
            Check if the hall sensors are delivering valid data
            - If not, go to SENSOR_ERROR state
            - If yes
                - Transition to CALIBRATION_FROM_FILE state
                - Indicate with single blink GREEN LED
            - Visualizer (permanent): Unchanged
        */
        CHECK_SENSORS,
        /*
            Failed to get valid hall sensor values
            - Reattempt SENSOR_RECONNECT after a delay
            - Visualizer (permanent): Solid RED LED
        */
        SENSOR_ERROR,
        /*
            Reattempt to reconnect to hall sensors
            - Visualizer (permanent): Unchanged
        */
        SENSOR_RECONNECT,
        /*
            Attempt to load calibration data from file
            - If successful
                - apply calibration and transition to RUNNING state
                - Visualizer: blinke once CYAN LED to indicate successful calibration load
            - If unsuccessful
                - transition to CALIBRATE_COLLECT state
                - Visualizer: blink once MAGENTA LED to indicate failed calibration load
            - Visualizer (permanent): Unchanged
        */
        CALIBRATION_FROM_FILE,
        /*
            Entry point for fresh calibration.
            Either during BOOT when CALIBRATION_FROM_FILE fails, or during RUNNING triggered by dual long button press.
            - Collect raw sensor data for calibration
                - On timeout
                    - transition to SENSOR_ERROR state
                    - Visualizer: blink once MAGENTA LED to indicate calibration collection timeout
                - On successful collection of enough samples
                    - transition to CALIBRATE_COMPUTE state
            - Visualizer (permanent): Solid spinner BLUE LED to indicate calibration
        */
        CALIBRATE_COLLECT,
        /*
            Perform calibration; must be done after CALIBRATE_COLLECT state
            - If calibration fails and no data from file
                - transition to RUNNING_WITHOUT_CALIBRATION state
                - Visualizer: blink twice MAGENTA LED to indicate calibration failure
            - If calibration succeeds and saving to file fails
                - transition to RUNNING state
                - Visualizer: blink three times MAGENTA LED to indicate calibration success but save failure
            - If calibration succeeds and saving to file succeeds
                - transition to RUNNING state
                - Visualizer: blink once GREEN LED to indicate successful calibration and save
            - Visualizer (permanent): Unchanged
        */
        CALIBRATE_COMPUTE,
        /*
            Normal operation state
            - Read raw sensor data, send to Core 1 for processing, and check for Core 1 response
            - TODO: Send HID data at fixed intervals > dt of Kalman
            - Visualizer (permanent): Solid LED_RUNNING_COLOR LED to indicate normal operation with calibration
        */
        RUNNING,
        /*
            Normal operation state without calibration.
            Identical to RUNNING state, but indicates that the system is running without calibration.
            - Visualizer (permanent): Solid ORANGE LED to indicate normal operation without calibration
        */
        RUNNING_WITHOUT_CALIBRATION,
        /*
            Low-power idle operation without LED visualizer.
            - Hall sensors run in low-power mode and are sampled at a reduced rate
            - Raw field changes restore the appropriate RUNNING state
            - Core 1 skips EKF processing until activity resumes
            - Visualizer (permanent): Off
        */
        SLEEP,
    };

    StateMachine(LEDController& led_controller, DipoleModel& dipole_model);

    State get_state();
    String get_state_name();
    uint32_t get_last_state_change_time_ms();
    void set_last_state_change_time_ms(uint32_t time_ms);
    uint32_t get_last_calibration_sample_time_ms();
    void set_last_calibration_sample_time_ms(uint32_t time_ms);
    uint32_t get_last_filtered_data_received_time_ms();
    void set_last_filtered_data_received_time_ms(uint32_t time_ms);
    Calibration::LoadState get_calibration_load_state();

    void enter_BOOT();
    void enter_CHECK_SENSORS();
    void enter_SENSOR_ERROR();
    void enter_SENSOR_RECONNECT();
    void enter_CALIBRATION_FROM_FILE();
    void enter_CALIBRATE_COLLECT();
    void enter_CALIBRATE_COMPUTE();
    void enter_RUNNING();
    void enter_RUNNING_WITHOUT_CALIBRATION();
    void enter_SLEEP();

    // We only handle some of the state transitions here,
    // as some transitions need several global data or functions
    // not worth passing all to the StateMachine class.
    // Those transitions are handled in main.cpp.
    void handle_BOOT(uint32_t now);
    void handle_CHECK_SENSORS(bool sensor_status);
    void handle_CALIBRATION_FROM_FILE();
    bool handle_CALIBRATE_COLLECT_partial(uint32_t now);
    void handle_CALIBRATE_COMPUTE();

private:
    LEDController& m_led_controller;
    DipoleModel& m_dipole_model;

    State m_state{State::BOOT};
    uint32_t m_last_state_change_time_ms{0};                   // Timestamp of the last state change
    uint32_t m_last_calibration_sample_time_ms{0};             // Timestamp of the last calibration sample collection
    uint32_t m_last_filtered_data_received_time_ms{0};         // Timestamp of the last filtered data received from Core 1
    uint32_t m_last_nonzero_filtered_data_received_time_ms{0}; // Timestamp of the last non-zero filtered data received from Core 1

    Calibration::LoadState m_calibration_load_state{Calibration::LoadState::NO_FILE_USING_DEFAULTS};

    bool change_state(State new_state);
};