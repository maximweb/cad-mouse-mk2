// =============================================================================
// HARDWARE
// Set once for the physical board and wiring. These values normally stay fixed.
// =============================================================================

// Button and LED pins
#define PIN_RIGHT_BTN D0
#define PIN_LEFT_BTN D2
#define PIN_LED_DATA D3
#define PIN_LED_LS D1

// Hall sensor power pins
#define PIN_MAG1_LS D10
#define PIN_MAG2_LS D9
#define PIN_MAG3_LS D8

// =============================================================================
// VISUALS
// LED colors, brightness, input glow, and power-transition effects.
// =============================================================================

#define LED_COUNT 8
#define LED_BRIGHTNESS 40                              // 0..255; base brightness for permanent LED effects
#define LED_BOOT_COLOR 0xFFFF00                        // Yellow
#define LED_ERROR_COLOR 0xFF0000                       // Red
#define LED_SUCCESS_COLOR 0x00FF00                     // Green
#define LED_CALIBRATION_COLOR 0x0000FF                 // Blue
#define LED_CALIBRATION_SUCCESS_COLOR 0x00FFFF         // Cyan
#define LED_CALIBRATION_FAILURE_COLOR 0xFF00FF         // Magenta
#define LED_RUNNING_COLOR 0xFFFFFF                     // White
#define LED_RUNNING_WITHOUT_CALIBRATION_COLOR 0xFF6600 // Orange

// Input glow effect during normal running
#define LED_USE_INPUT_GLOW_EFFECT true    // Set false to disable LED input glow effect
#define LED_INPUT_GLOW_COLOR 0x00FFFF     // Cyan
#define LED_INPUT_GLOW_MIN_BRIGHTNESS 10  // 0..255; floor for input glow, must be <= LED_BRIGHTNESS and MAX
#define LED_INPUT_GLOW_MAX_BRIGHTNESS 255 // 0..255; peak input-glow brightness, must be >= LED_BRIGHTNESS and MIN

// Power-transition effects
#define LED_FADE_OFF_DURATION_MS 2000 // Time it takes to fade LEDs off before sleep
#define LED_FADE_ON_DURATION_MS 1000  // Time it takes to fade LEDs on after wake

// =============================================================================
// RUNTIME AND POWER MANAGEMENT
// State transitions, inactivity, and low-power behavior.
// =============================================================================

#define BOOT_DELAY_MS 1000                        // Delay before the initial sensor check
#define SENSOR_RECONNECT_DELAY_MS 1000            // Delay before retrying a failed sensor connection
#define RUNNING_STATE_INACTIVITY_TIMEOUT_MS 60000 // 60 seconds until LEDs turned off due to inactivity

#define SLEEP_SAMPLE_INTERVAL_MS 100      // Poll sensors and buttons at 10 Hz while waiting for motion
#define SLEEP_WAKE_THRESHOLD 5.0f         // > 0; per-axis field change required to resume normal processing
#define SLEEP_BASELINE_ALPHA 0.02f        // 0..1; baseline adaptation rate while idle
#define SLEEP_WAKE_GRACE_MS 1000          // Grace period preventing immediate sleep after wake
#define SLEEP_SENSOR_ERROR_TIMEOUT_MS 500 // Timeout adapted to the lower sensor update rate

// Board-specific timeout for filtered data reception from Core 1
#ifdef BOARD_RP2350
#define RUNNING_STATE_READ_ERROR_TIMEOUT_MS 50
#else
#ifdef BOARD_RP2040
#define RUNNING_STATE_READ_ERROR_TIMEOUT_MS 100
#else
#define RUNNING_STATE_READ_ERROR_TIMEOUT_MS 200
#endif
#endif

// =============================================================================
// INPUT
// =============================================================================

#define BUTTON_COMBO_WINDOW_MS 500 // Time window for recognizing a combined long press

// =============================================================================
// CALIBRATION
// Sample collection, acceptance criteria, and fit bounds.
// =============================================================================

#define CALIBRATION_SAMPLE_COUNT 100                                                               // 10..255; samples collected for calibration
#define CALIBRATION_SAMPLE_DELAY_MS 20                                                             // > 0; interval between collected samples
#define CALIBRATION_SAMPLE_TIMEOUT_MS (CALIBRATION_SAMPLE_COUNT * CALIBRATION_SAMPLE_DELAY_MS * 5) // 5x nominal collection time
#define CALIBRATION_DATA_STD_THRESHOLD 0.5f                                                        // >= 0; maximum accepted standard deviation for stable samples

// Fit bounds constrain the calibration optimizer to physically plausible values.
// Magnetic moments use one symmetric magnitude bound, while assembly offsets
// use separate lower and upper bounds for each translation and rotation axis.
#define CALIBRATION_FIT_MOMENT_BOUNDS 0.5f // > 0; symmetric +/- fit bound in A/m^2
#define CALIBRATION_FIT_MOMENT_MIN 0.05f   // >= 0 and <= BOUNDS; minimum accepted magnitude in A/m^2

#define CALIBRATION_FIT_X_MIN -1.0f  // mm; must be <= X_MAX
#define CALIBRATION_FIT_X_MAX 1.0f   // mm; must be >= X_MIN
#define CALIBRATION_FIT_Y_MIN -1.0f  // mm; must be <= Y_MAX
#define CALIBRATION_FIT_Y_MAX 1.0f   // mm; must be >= Y_MIN
#define CALIBRATION_FIT_Z_MIN -1.0f  // mm; must be <= Z_MAX
#define CALIBRATION_FIT_Z_MAX 1.0f   // mm; must be >= Z_MIN
#define CALIBRATION_FIT_RX_MIN -1.0f // degrees; must be <= RX_MAX
#define CALIBRATION_FIT_RX_MAX 1.0f  // degrees; must be >= RX_MIN
#define CALIBRATION_FIT_RY_MIN -1.0f // degrees; must be <= RY_MAX
#define CALIBRATION_FIT_RY_MAX 1.0f  // degrees; must be >= RY_MIN
#define CALIBRATION_FIT_RZ_MIN -1.0f // degrees; must be <= RZ_MAX
#define CALIBRATION_FIT_RZ_MAX 1.0f  // degrees; must be >= RZ_MIN

// =============================================================================
// DIPOLE MODEL
// =============================================================================

// Initial magnetic moment for each magnet. Also used as the calibration start
// value when no stored calibration is available.
#define DIPOLE_MODEL_MAGNETIC_MOMENT_DEFAULT 0.18f // A*m^2

// =============================================================================
// EXTENDED KALMAN FILTER
// =============================================================================

#define EKF_PROCESS_NOISE_STD 1.0f // > 0; process noise standard deviation; higher = more responsive/noisy
#define EKF_SENSOR_NOISE_STD 5.0f  // > 0; sensor noise standard deviation; higher = smoother/less sensor trust

// Jacobian strategy: fully recomputed every update step.
// 0: Fully numeric finite differences
// 1: Hybrid (analytic translation dB/dx,dB/dy,dB/dz + numeric rotation dB/drx,dB/dry,dB/drz)
#define EKF_JACOBIAN_MODE 1

// =============================================================================
// POSTPROCESSING
// Normalization, deadzones, and 6DoF dominant-axis isolation.
// =============================================================================
//
// Physics model assumes (USB port facing away from user):
// - x: right(+ / MAX), left(- / MIN)
// - y: forward(+ / MAX), backward(- / MIN)
// - z: up(+ / MAX), down(- / MIN)
// - rx: roll backward(+ / MAX), roll forward(- / MIN)
// - ry: pitch right(+ / MAX), pitch left(- / MIN)
// - rz: yaw left(+ / MAX), yaw right(- / MIN)
//
// HID report mapping flips y, z, ry, and rz signs for driver support. The
// normalization limits below keep the physical MIN/MAX direction convention.
// To tune asymmetric movement, decrease the corresponding positive MAX or
// negative MIN value. For example, decreasing NORMALIZATION_Z_MAX makes upward
// movement more sensitive.
#define NORMALIZATION_X_MAX 1.7f  // Positive x movement for normalized +1, in mm
#define NORMALIZATION_X_MIN 1.7f  // Magnitude of negative x movement for normalized -1, in mm
#define NORMALIZATION_Y_MAX 1.7f  // Positive y movement for normalized +1, in mm
#define NORMALIZATION_Y_MIN 1.7f  // Magnitude of negative y movement for normalized -1, in mm
#define NORMALIZATION_Z_MAX 1.0f  // Positive z movement for normalized +1, in mm; lower for more sensitivity
#define NORMALIZATION_Z_MIN 1.5f  // Magnitude of negative z movement for normalized -1, in mm
#define NORMALIZATION_RX_MAX 6.5f // Positive rx rotation for normalized +1, in degrees
#define NORMALIZATION_RX_MIN 6.5f // Magnitude of negative rx rotation for normalized -1, in degrees
#define NORMALIZATION_RY_MAX 6.5f // Positive ry rotation for normalized +1, in degrees
#define NORMALIZATION_RY_MIN 6.5f // Magnitude of negative ry rotation for normalized -1, in degrees
#define NORMALIZATION_RZ_MAX 5.5f // Positive rz rotation for normalized +1, in degrees
#define NORMALIZATION_RZ_MIN 5.5f // Magnitude of negative rz rotation for normalized -1, in degrees

#define DEADZONE_TRANSLATION_THRESHOLD 0.05f // Normalized x/y/z magnitude; 0.05 = 5% deadzone
#define DEADZONE_ROTATION_THRESHOLD 0.05f    // Normalized rx/ry/rz magnitude; 0.05 = 5% deadzone
#define ISOLATION_POWER 3.0f                 // > 0; 1=no curve, 3=cubic; 1, 2, 3, 0.5 optimized

// =============================================================================
// HID OUTPUT
// =============================================================================

// Positive HID logical limit; the report range is [-AXIS_LIMIT, +AXIS_LIMIT].
// Must stay within the signed 16-bit range; no runtime clamp is applied.
#define AXIS_LIMIT 350 // 1..32767; higher values increase maximum on-screen velocity

// =============================================================================
// DEBUGGING AND PROFILING
// =============================================================================

// Profiling levels: 0 = off, 1 = lightweight throughput telemetry, 2 = detailed sections.
#define PERFORMANCE_PROFILING_LEVEL 0 // 0..2; values >2 behave like level 2

#if PERFORMANCE_PROFILING_LEVEL > 0
#define ENABLE_PERFORMANCE_PROFILING 1
#endif

#define PERFORMANCE_PRINT_INTERVAL_MS 3000

// Central debug switches (0 = off, 1 = on)
#define DEBUG_MAIN_SERIAL 0
#define DEBUG_MAIN_PRINT_CORE1_DURATION 0
#define DEBUG_STATE_MACHINE_SERIAL 0
#define DEBUG_CALIBRATION_SERIAL 0
#define DEBUG_DIPOLE_MODEL_SERIAL 0
#define DEBUG_KALMAN_FILTER_SERIAL 0
