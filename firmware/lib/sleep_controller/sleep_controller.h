#pragma once

#include <Arduino.h>

class HallSensorController;
class LEDController;

class SleepController {
public:
    enum class Result {
        WAITING,
        WAKE_REQUESTED,
        SENSOR_TIMEOUT,
    };

    Result update(uint32_t now_ms, bool host_activity_recent, HallSensorController& hall_controller, LEDController& led_controller);
    bool wakeGraceActive(uint32_t now_ms) const;

private:
    bool m_session_initialized{false};
    bool m_baseline_valid{false};
    uint32_t m_last_sensor_success_time_ms{0};
    uint32_t m_last_wake_time_ms{0};
    float m_sensor_baseline[9]{};

    void resetSession();
};