#include "sleep_controller.h"

#include <math.h>

#include "config.h"
#include "hall_controller.h"
#include "led_controller.h"
#include "power_manager.h"

SleepController::Result SleepController::update(uint32_t now_ms, bool host_activity_recent, HallSensorController& hall_controller, LEDController& led_controller)
{
    hall_controller.enterLowPowerMode();
    if (hall_controller.isInLowPowerMode() && !led_controller.is_fading()) {
        PowerManager::enterSleep();
    }

    if (!m_session_initialized) {
        m_session_initialized = true;
        m_baseline_valid = false;
        m_last_sensor_success_time_ms = now_ms;
    }

    bool motion_detected = false;
    if (PowerManager::sleepActive() || now_ms - m_last_sensor_success_time_ms >= SLEEP_SAMPLE_INTERVAL_MS) {
        float raw_sensor_data[9];
        const uint8_t sensor_status = hall_controller.read(raw_sensor_data);

        if (sensor_status == HALL_STATUS_OK) {
            m_last_sensor_success_time_ms = now_ms;

            if (!m_baseline_valid) {
                for (int index = 0; index < 9; ++index) {
                    m_sensor_baseline[index] = raw_sensor_data[index];
                }
                m_baseline_valid = true;
            }
            else {
                for (int index = 0; index < 9; ++index) {
                    if (fabsf(raw_sensor_data[index] - m_sensor_baseline[index]) >= SLEEP_WAKE_THRESHOLD) {
                        motion_detected = true;
                        break;
                    }
                }

                if (!motion_detected) {
                    for (int index = 0; index < 9; ++index) {
                        m_sensor_baseline[index] += SLEEP_BASELINE_ALPHA * (raw_sensor_data[index] - m_sensor_baseline[index]);
                    }
                }
            }
        }
    }

    if (now_ms - m_last_sensor_success_time_ms > SLEEP_SENSOR_ERROR_TIMEOUT_MS) {
        resetSession();
        return Result::SENSOR_TIMEOUT;
    }

    if (!motion_detected && !host_activity_recent) {
        return Result::WAITING;
    }

    PowerManager::exitSleep();
    hall_controller.enterFastMode();
    m_last_wake_time_ms = now_ms;
    resetSession();
    return Result::WAKE_REQUESTED;
}

bool SleepController::wakeGraceActive(uint32_t now_ms) const
{
    return now_ms - m_last_wake_time_ms <= SLEEP_WAKE_GRACE_MS;
}

void SleepController::resetSession()
{
    m_session_initialized = false;
    m_baseline_valid = false;
}