#pragma once

#include "config.h"

#include <Adafruit_TinyUSB.h>
#include <Arduino.h>

class HIDController {
public:
    struct ReportAxes {
        int16_t x;
        int16_t y;
        int16_t z;
        int16_t rx;
        int16_t ry;
        int16_t rz;
    };

    struct ReportButtons {
        uint16_t bits;
    };

    bool begin();
    void task();
    void sendReport(float filtered_state[12], uint16_t buttons, bool forceUpdate);

    uint32_t get_last_report_time_ms();

private:
    Adafruit_USBD_HID m_hid;

    uint32_t m_last_sent_time_ms{0}; // Timestamp of the last sent report
    ReportAxes m_report_axes;
    ReportButtons m_report_buttons;

    ReportAxes makeReportAxes(float filtered_state[12]);
    ReportButtons makeReportButtons(uint16_t buttons);

    bool axesChanged(const ReportAxes& new_axes, const uint32_t now);
    bool buttonsChanged(const ReportButtons& new_buttons);
};