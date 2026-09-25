#pragma once

namespace PowerManager {

// Disable board-level loads that are not used by the application.
void begin();

// Reduce/restore RP2040 or RP2350 clocks while retaining USB, timer, I2C1
// and memory. These functions are no-ops on other targets and safe to repeat.
void enterSleep();
void exitSleep();
bool sleepActive();

} // namespace PowerManager
