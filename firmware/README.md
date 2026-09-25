# CAD Mouse MK2 - Alternative Firmware

This is an alternative implementation for the awesome CAD Mouse MK2
([Code](https://github.com/sb-ocr/cad-mouse-mk2), [YouTube Video](https://youtu.be/62xlzGs8LXA), [Instructables](https://www.instructables.com/CAD-Mouse-MK2-a-6DoF-Space-Mouse-Using-Magnets), [preassembled PCB](https://ocrlab.myshopify.com/products/sensor-board-cad-mouse-mk2))

*I am not affiliated with the original author in any kind.*

I opted for a reimplementation rather than a fork because I wanted to learn how it works. Yet, several features closely follow the original implementation.

## Hardware

I chose a (pin compatible) SeeedStudio XIAO RP2350 instead of the original RP2040 because the former has

- generally more processing power
- hardware accelerated floating point capabilities

The PCB is a preassembled one I bought. Only hickup was a broken LED.
Hence my config.h contains setting for 7 instead of original 8 LEDs.

## PlatformIO Environments

The project now provides two PlatformIO environments:

- `RP2040`
- `RP2350`

`RP2350` is currently the default environment in `platformio.ini`.

## Performance Status

- **RP2040 (after recent performance improvements):**
  - Filter runtime is roughly **6.6 ms** on average.
  - HID report interval is set to **7 ms**, which corresponds to about **142.9 Hz**.
- **RP2350 (before recent performance improvements):**
  - HID report interval was set to **4 ms** (**250 Hz**), with a noted Core1 roundtrip time of about **2 ms** (from `config.h` comment).
  - Post-improvement numbers on RP2350 are not measured yet.

## Features

### Concept

The core idea was to improve the sensor processing / motion engine with

- a physics based dipole model
- in combination with an extended Kalman filter
- using dual-core capabilities of the MCU for the filtering

#### Dipole Physics Model

- The physics model represents each magnet by a dipole equation at the center of each magnet.
- The Hall-sensors are assumed to be in an equilateral triangle in a x-y-plane.
- The magnets are assumed to be in an equilateral triangle
  - in a x-y-plane above the sensors in neutral position
  - fixed to each other but transalted or rotated around the springs center
- [An external test recording raw readings at various known fixed positions revealed that a single dipole per magnet is sufficient to predict raw readings](https://github.com/sb-ocr/cad-mouse-mk2/issues/19#issuecomment-4987111997)

#### Kalman Filter and Postprocessing

- The sensor data are read in main loop and sent to Core1 for filtering
- an Extended Kalman filter based on the physics model then filter and predicts smooth x,y,z,rx,ry,rz state, as well as their (angular) velocities vx,vy,vz,vrx,vry,vrz
- Postprocessing does
  - normalize the translation (mm) and rotation (deg / rad) to [-1,1] -> still need to expose the limits
  - translation and rotation vectors are independently deadzones by calculating their vector magnitude, so we don't have any jitter in neutral position
  - to isolate movements, the combined state vector is cubed and renomarlized to pronounce movements of a dominant axis

### Calibration

Calibration is done in firmware by

- collecting and averaging sensor data
- fitting each magnets dipole magnetic moment via the dipole physics engine to best represent the Hall sensor magnetic field data
- fitting tiny x,y,z,rx,ry,rz offsets due to slight assembly inacuracies also using the dipole physics engine
- persisting the calibration results to file with LittleFS

Initial start triggers calibration and attempts to store it to LittleFS. Consecutive calibrations can be manually triggered by long press of both buttons simultaneously.

### LED Color Codes

All firmware LED colors use `0xRRGGBB` values from `firmware/include/config.h`.
Permanent effects describe the current state and remain active until another
state changes them. Queued animations are one-time event notifications and
restore the relevant permanent effect afterwards.

#### Permanent Colors and Effects

| Constant                                | Hex code   | Color  | Effect                     | Meaning                              |
| --------------------------------------- | ---------- | ------ | -------------------------- | ------------------------------------ |
| `LED_BOOT_COLOR`                        | `0xFFFF00` | Yellow | Solid                      | Boot state                           |
| `LED_ERROR_COLOR`                       | `0xFF0000` | Red    | Full spinner               | Sensor error                         |
| `LED_CALIBRATION_COLOR`                 | `0x0000FF` | Blue   | Full spinner               | Calibration in progress              |
| `LED_RUNNING_COLOR`                     | `0xFFFFFF` | White  | Solid; optional input glow | Normal running with calibration      |
| `LED_RUNNING_WITHOUT_CALIBRATION_COLOR` | `0xFF6600` | Orange | Solid                      | Running without calibration          |
| `LED_INPUT_GLOW_COLOR`                  | `0x00FFFF` | Cyan   | Dynamic input glow         | Input feedback during normal running |
| internal `0x000000`                     | `0x000000` | Black  | Off                        | LEDs switched off, including sleep   |

#### One-Time Queued Animations

All entries below use `queue_blinking_animation` with 200 ms on and 200 ms
off durations. The blink count is part of the event code and should remain
unique for different events that use the same color.

| Constant                        | Hex code   | Color   | Blinks | Event                                                          |
| ------------------------------- | ---------- | ------- | -----: | -------------------------------------------------------------- |
| `LED_SUCCESS_COLOR`             | `0x00FF00` | Green   |      1 | Sensor check succeeded                                         |
| `LED_CALIBRATION_SUCCESS_COLOR` | `0x00FFFF` | Cyan    |      1 | Calibration file loaded successfully                           |
| `LED_CALIBRATION_FAILURE_COLOR` | `0xFF00FF` | Magenta |      1 | Filesystem initialization failed at startup                    |
| `LED_CALIBRATION_FAILURE_COLOR` | `0xFF00FF` | Magenta |      2 | Calibration file could not be loaded; fresh calibration starts |
| `LED_CALIBRATION_FAILURE_COLOR` | `0xFF00FF` | Magenta |      3 | Calibration sample collection timed out                        |
| `LED_CALIBRATION_FAILURE_COLOR` | `0xFF00FF` | Magenta |      4 | Fresh calibration succeeded, but saving failed                 |
| `LED_CALIBRATION_FAILURE_COLOR` | `0xFF00FF` | Magenta |      5 | Fresh calibration failed; existing calibration remains active  |
| `LED_CALIBRATION_FAILURE_COLOR` | `0xFF00FF` | Magenta |      6 | Fresh calibration failed; no calibration is available          |
| `LED_SUCCESS_COLOR`             | `0x00FF00` | Green   |      2 | Fresh calibration succeeded and was saved                      |

The blink code is unique for every event within a color. Filesystem and
calibration-file failures intentionally use the same Magenta family because
the filesystem is currently used exclusively for calibration persistence.
`LED_CALIBRATION_SUCCESS_COLOR` and `LED_INPUT_GLOW_COLOR` also share
`0x00FFFF`, but one is a queued event and the other is a permanent dynamic
effect.

### Quick Tuning Guide

Main knobs in `firmware/include/config.h`:

- `EKF_PROCESS_NOISE_STD`: higher = more responsive, lower = smoother
- `EKF_SENSOR_NOISE_STD`: higher = smoother/less reactive, lower = more direct/more noisy
- `NORMALIZATION_*_MAX`: higher = less sensitive axis, lower = more sensitive axis
- `DEADZONE_*_THRESHOLD`: higher = less idle jitter, lower = finer micro-movements
- `ISOLATION_POWER`: higher = stronger dominant-axis isolation

For EKF tuning, set deadzone temporarily very small (recommended: `0.005` to `0.02`) so neutral jitter/spikes stay visible.

Current normalization is symmetric (absolute max only, mapped to `[-1, 1]`).
If one direction needs different scaling than the other, we may need separate `+/-` limits (for example for `Z`: push down `-Z` vs pull up `+Z` on lightweight builds without base weights).

### ToDo

The filter settings and normalization limits may need further tuning.
