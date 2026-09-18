#include "hid_controller.h"

namespace {
    const uint8_t hid_report_descriptor[] = {
      HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),             // USAGE_PAGE (Generic Desktop)
      HID_USAGE(HID_USAGE_DESKTOP_MULTI_AXIS_CONTROLLER), // USAGE (Multi-axis Controller)
      HID_COLLECTION(HID_COLLECTION_APPLICATION),         // COLLECTION (Application)

      HID_COLLECTION(HID_COLLECTION_PHYSICAL),           // COLLECTION (Physical)
      HID_REPORT_ID(0x01)                                // REPORT_ID (1)
      HID_LOGICAL_MIN_N(-AXIS_LIMIT, 2),                 // LOGICAL_MINIMUM
      HID_LOGICAL_MAX_N(AXIS_LIMIT, 2),                  // LOGICAL_MAXIMUM
      HID_USAGE(HID_USAGE_DESKTOP_X),                    // USAGE (X)
      HID_USAGE(HID_USAGE_DESKTOP_Y),                    // USAGE (Y)
      HID_USAGE(HID_USAGE_DESKTOP_Z),                    // USAGE (Z)
      HID_USAGE(HID_USAGE_DESKTOP_RX),                   // USAGE (Rx)
      HID_USAGE(HID_USAGE_DESKTOP_RY),                   // USAGE (Ry)
      HID_USAGE(HID_USAGE_DESKTOP_RZ),                   // USAGE (Rz)
      HID_REPORT_SIZE(16),                               // REPORT_SIZE (16)
      HID_REPORT_COUNT(6),                               // REPORT_COUNT (6)
      HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE), // INPUT (Data,Var,Abs)
      HID_COLLECTION_END,                                // END_COLLECTION

      HID_COLLECTION(HID_COLLECTION_PHYSICAL),            // COLLECTION (Physical)
      HID_REPORT_ID(0x03)                                 // REPORT_ID (3)
      HID_USAGE_PAGE(HID_USAGE_PAGE_BUTTON),              // USAGE_PAGE (Button)
      HID_USAGE_MIN(1),                                   // USAGE_MINIMUM (Button 1)
      HID_USAGE_MAX(2),                                   // USAGE_MAXIMUM (Button 2)
      HID_LOGICAL_MIN(0),                                 // LOGICAL_MINIMUM (0)
      HID_LOGICAL_MAX(1),                                 // LOGICAL_MAXIMUM (1)
      HID_REPORT_SIZE(1),                                 // REPORT_SIZE (1)
      HID_REPORT_COUNT(2),                                // REPORT_COUNT (2)
      HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),  // INPUT (Data,Var,Abs)
      HID_REPORT_COUNT(14),                               // REPORT_COUNT (14) padding
      HID_INPUT(HID_CONSTANT | HID_ARRAY | HID_ABSOLUTE), // INPUT (Const,Array,Abs)
      HID_COLLECTION_END,                                 // END_COLLECTION

      HID_COLLECTION_END, // END_COLLECTION
    };
}

bool HIDController::begin()
{
    if (!TinyUSBDevice.isInitialized()) {
        if (!TinyUSBDevice.begin(0)) {
            return false; // Failed to initialize TinyUSBDevice
        }
    }

    // Initialize the HID device with the report descriptor
    m_hid.setReportDescriptor(hid_report_descriptor, sizeof(hid_report_descriptor));
    m_hid.setPollInterval(HID_REPORT_INTERVAL_MS);
    m_hid.begin();

    // If already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
    if (TinyUSBDevice.mounted()) {
        TinyUSBDevice.detach();
        delay(10);
        TinyUSBDevice.attach();
    }

    m_report_axes.x = 0;
    m_report_axes.y = 0;
    m_report_axes.z = 0;
    m_report_axes.rz = 0;
    m_report_axes.rx = 0;
    m_report_axes.ry = 0;

    m_report_buttons.bits = 0;

    return true;
}

void HIDController::task()
{
    TinyUSBDevice.task();
}

void HIDController::sendReport(float filtered_state[12], uint16_t buttons)
{
    // Send HID report
    task(); // Ensure TinyUSBDevice task is called to handle USB events

    // Skip sending if not mounted
    if (!TinyUSBDevice.mounted()) {
        return;
    }

    // Remote wakeup
    if (TinyUSBDevice.suspended() && buttons) {
        // Wake up host if we are in suspend mode
        // and REMOTE_WAKEUP feature is enabled by host
        TinyUSBDevice.remoteWakeup();
    }

    if (!m_hid.ready()) {
        return; // HID device not ready to send report
    }

    const uint32_t now = millis();

    // Button report only sent when changed, regardless of HID_REPORT_INTERVAL_MS
    ReportButtons new_buttons = makeReportButtons(buttons);
    bool buttons_changed = buttonsChanged(new_buttons);
    if (buttons_changed) {
        m_report_buttons = new_buttons;
        m_hid.sendReport(0x03, &m_report_buttons, sizeof(m_report_buttons));
        task();
        m_last_sent_time_ms = now; // Update the timestamp of the last sent report
    }

    // Check if enough time has passed since the last axes report was sent
    if (now - m_last_sent_time_ms < HID_REPORT_INTERVAL_MS) {
        return;
    }

    ReportAxes new_axes = makeReportAxes(filtered_state);
    if (!axesChanged(new_axes, now)) {
        return;
    }

    // Sent axes report
    m_report_axes = new_axes;
    m_hid.sendReport(0x01, &m_report_axes, sizeof(m_report_axes));
    task();

    m_last_sent_time_ms = now; // Update the timestamp of the last sent report
}

uint32_t HIDController::get_last_report_time_ms()
{
    return m_last_sent_time_ms;
}

HIDController::ReportAxes HIDController::makeReportAxes(float filtered_state[12])
{
    // Seems like some software prefers
    // y-axis points towards the user
    // z-axis points downwards
    ReportAxes report_axes;
    report_axes.x = static_cast<int16_t>(filtered_state[0] * AXIS_LIMIT);
    report_axes.y = static_cast<int16_t>(filtered_state[1] * -AXIS_LIMIT);
    report_axes.z = static_cast<int16_t>(filtered_state[2] * -AXIS_LIMIT);
    report_axes.rx = static_cast<int16_t>(filtered_state[3] * AXIS_LIMIT);
    report_axes.ry = static_cast<int16_t>(filtered_state[4] * -AXIS_LIMIT);
    report_axes.rz = static_cast<int16_t>(filtered_state[5] * -AXIS_LIMIT);
    return report_axes;
}

HIDController::ReportButtons HIDController::makeReportButtons(uint16_t buttons)
{
    ReportButtons report_buttons;
    report_buttons.bits = buttons & 0x0003;
    return report_buttons;
}

bool HIDController::axesChanged(const ReportAxes& new_axes, const uint32_t now)
{
    if (new_axes.x != m_report_axes.x ||   //
        new_axes.y != m_report_axes.y ||   //
        new_axes.z != m_report_axes.z ||   //
        new_axes.rx != m_report_axes.rx || //
        new_axes.ry != m_report_axes.ry || //
        new_axes.rz != m_report_axes.rz) {
        return true;
    }
    else if (now - m_last_sent_time_ms > 50 && (new_axes.x != 0 ||  //
                                                new_axes.y != 0 ||  //
                                                new_axes.z != 0 ||  //
                                                new_axes.rx != 0 || //
                                                new_axes.ry != 0 || //
                                                new_axes.rz != 0)) {
        return true;
    }
    return false;
}

bool HIDController::buttonsChanged(const ReportButtons& new_buttons)
{
    return new_buttons.bits != m_report_buttons.bits;
}