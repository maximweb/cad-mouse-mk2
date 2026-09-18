#include <Arduino.h>

#include "config.h"
#include "power_manager.h"

#if defined(BOARD_RP2040) || defined(BOARD_RP2350)
#include <hardware/clocks.h>
#include <hardware/regs/clocks.h>
#include <hardware/structs/clocks.h>
#endif

namespace PowerManager {

#if defined(BOARD_RP2040) || defined(BOARD_RP2350)
namespace {

#ifdef BOARD_RP2350
// XIAO RP2350 onboard GPIOs without Arduino-Pico pin macros.
// The addressable RGB LED has no power-enable GPIO, so hold its data input low.
constexpr uint8_t ONBOARD_RGB_DATA_PIN = 22u;
constexpr uint8_t BATTERY_SENSE_ENABLE_PIN = 19u; // Active high
#endif

bool g_sleep_clocks_active = false;
uint32_t g_awake_sys_clock_hz = 0;
uint32_t g_awake_peri_clock_hz = 0;
uint32_t g_awake_sleep_en0 = 0;
uint32_t g_awake_sleep_en1 = 0;

#ifdef BOARD_RP2040
// The sleep clock masks take effect only while both cores execute WFI/WFE.
// Keep USB device operation, the timer used by delay(), I2C1 used by Wire,
// GPIO, and every memory bank that can hold application state.
constexpr uint32_t SLEEP_EN0 =
  CLOCKS_SLEEP_EN0_CLK_SYS_SRAM3_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_SRAM2_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_SRAM1_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_SRAM0_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_SIO_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_PLL_USB_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_PLL_SYS_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_PADS_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_VREG_AND_CHIP_RESET_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_IO_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_I2C1_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_BUSFABRIC_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_BUSCTRL_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_CLOCKS_BITS;

constexpr uint32_t SLEEP_EN1 =
  CLOCKS_SLEEP_EN1_CLK_SYS_XOSC_BITS |
  CLOCKS_SLEEP_EN1_CLK_SYS_XIP_BITS |
  CLOCKS_SLEEP_EN1_CLK_SYS_TIMER_BITS |
  CLOCKS_SLEEP_EN1_CLK_SYS_SYSCFG_BITS |
  CLOCKS_SLEEP_EN1_CLK_SYS_SRAM5_BITS |
  CLOCKS_SLEEP_EN1_CLK_SYS_SRAM4_BITS |
  CLOCKS_SLEEP_EN1_CLK_USB_USBCTRL_BITS |
  CLOCKS_SLEEP_EN1_CLK_SYS_USBCTRL_BITS;
#else
// RP2350 has 63 clock destinations split over two sleep-enable words. Keep
// both USB clocks, both timer blocks and their tick sources, I2C1 used by
// Wire1, GPIO, clock/power control, XIP, and every SRAM bank.
constexpr uint32_t SLEEP_EN0 =
  CLOCKS_SLEEP_EN0_CLK_SYS_CLOCKS_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_ACCESSCTRL_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_BUSCTRL_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_BUSFABRIC_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_I2C1_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_IO_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_PADS_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_PLL_USB_BITS |
  CLOCKS_SLEEP_EN0_CLK_REF_POWMAN_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_POWMAN_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_RESETS_BITS |
  CLOCKS_SLEEP_EN0_CLK_SYS_SIO_BITS;

constexpr uint32_t SLEEP_EN1 =
  CLOCKS_SLEEP_EN1_CLK_SYS_SRAM0_BITS |
  CLOCKS_SLEEP_EN1_CLK_SYS_SRAM1_BITS |
  CLOCKS_SLEEP_EN1_CLK_SYS_SRAM2_BITS |
  CLOCKS_SLEEP_EN1_CLK_SYS_SRAM3_BITS |
  CLOCKS_SLEEP_EN1_CLK_SYS_SRAM4_BITS |
  CLOCKS_SLEEP_EN1_CLK_SYS_SRAM5_BITS |
  CLOCKS_SLEEP_EN1_CLK_SYS_SRAM6_BITS |
  CLOCKS_SLEEP_EN1_CLK_SYS_SRAM7_BITS |
  CLOCKS_SLEEP_EN1_CLK_SYS_SRAM8_BITS |
  CLOCKS_SLEEP_EN1_CLK_SYS_SRAM9_BITS |
  CLOCKS_SLEEP_EN1_CLK_SYS_SYSCFG_BITS |
  CLOCKS_SLEEP_EN1_CLK_REF_TICKS_BITS |
  CLOCKS_SLEEP_EN1_CLK_SYS_TICKS_BITS |
  CLOCKS_SLEEP_EN1_CLK_SYS_TIMER0_BITS |
  CLOCKS_SLEEP_EN1_CLK_SYS_TIMER1_BITS |
  CLOCKS_SLEEP_EN1_CLK_SYS_USBCTRL_BITS |
  CLOCKS_SLEEP_EN1_CLK_USB_BITS |
  CLOCKS_SLEEP_EN1_CLK_SYS_WATCHDOG_BITS |
  CLOCKS_SLEEP_EN1_CLK_SYS_XIP_BITS |
  CLOCKS_SLEEP_EN1_CLK_SYS_XOSC_BITS;
#endif

void setOutputLevel(uint8_t pin, uint8_t level)
{
    pinMode(pin, OUTPUT);
    digitalWrite(pin, level);
}

} // namespace
#endif

void begin()
{
#ifdef BOARD_RP2040
    setOutputLevel(NEOPIXEL_POWER, LOW); // Active high
    setOutputLevel(PIN_NEOPIXEL, LOW);
    // The onboard status RGB LED is active low.
    setOutputLevel(PIN_LED_G, HIGH);
    setOutputLevel(PIN_LED_R, HIGH);
    setOutputLevel(PIN_LED_B, HIGH);
#elif defined(BOARD_RP2350)
    setOutputLevel(ONBOARD_RGB_DATA_PIN, LOW);
    setOutputLevel(LED_BUILTIN, HIGH); // Yellow user LED is active low
    setOutputLevel(BATTERY_SENSE_ENABLE_PIN, LOW);
#endif
}

void enterSleep()
{
#if defined(BOARD_RP2040) || defined(BOARD_RP2350)
    if (g_sleep_clocks_active) {
        return;
    }

    g_awake_sys_clock_hz = clock_get_hz(clk_sys);
    g_awake_peri_clock_hz = clock_get_hz(clk_peri);
#ifdef BOARD_RP2040
    g_awake_sleep_en0 = clocks_hw->sleep_en0;
    g_awake_sleep_en1 = clocks_hw->sleep_en1;
#else
    g_awake_sleep_en0 = clocks_hw->sleep_en[0];
    g_awake_sleep_en1 = clocks_hw->sleep_en[1];
#endif

set_sys_clock_48mhz();

#ifdef BOARD_RP2040
    clocks_hw->sleep_en0 = SLEEP_EN0;
    clocks_hw->sleep_en1 = SLEEP_EN1;
#else
    clocks_hw->sleep_en[0] = SLEEP_EN0;
    clocks_hw->sleep_en[1] = SLEEP_EN1;
#endif
    g_sleep_clocks_active = true;
#endif
}

void exitSleep()
{
#if defined(BOARD_RP2040) || defined(BOARD_RP2350)
    if (!g_sleep_clocks_active) {
        return;
    }

#ifdef BOARD_RP2040
    clocks_hw->sleep_en0 = g_awake_sleep_en0;
    clocks_hw->sleep_en1 = g_awake_sleep_en1;
#else
    clocks_hw->sleep_en[0] = g_awake_sleep_en0;
    clocks_hw->sleep_en[1] = g_awake_sleep_en1;
#endif
    set_sys_clock_khz((g_awake_sys_clock_hz + 999u) / 1000u, true);

    // set_sys_clock_khz() deliberately moves clk_peri to a safe 48 MHz source.
    // Put it back on clk_sys so I2C and the other peripherals regain their
    // original active-state rate after wake.
    clock_configure(
      clk_peri,
      0,
      CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
      clock_get_hz(clk_sys),
      g_awake_peri_clock_hz);
    g_sleep_clocks_active = false;
#endif
}

bool sleepActive()
{
    return g_sleep_clocks_active;
}

} // namespace PowerManager
