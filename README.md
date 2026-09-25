# CAD Mouse MK2

Watch the build video ↓

[<img src="./images/CAD_Mouse_MK2_Thumbnail.jpg">](https://youtu.be/62xlzGs8LXA)

This is the second iteration of my DIY CAD Mouse, rebuilt to behave like a real 6DoF controller. It uses a custom PCB with three magnetic sensors, a 3D printed spring, and a redesigned enclosure that is smaller and easier to build.

## Firmware

The firmware reads the three magnetic sensors, estimates the full
6DoF pose with a dipole physics model and extended Kalman filter, and uses both MCU
cores to separate sensor input from filtering. It supports persistent
calibration data and runs on both RP2040 and RP2350 boards.

See the [firmware documentation](./firmware/README.md) for implementation
details, tuning guidance, and LED status codes.

## Hardware

  Build instructions → [Instructables](https://www.instructables.com/CAD-Mouse-MK2-a-6DoF-Space-Mouse-Using-Magnets)

<sub>⚠️ There have been several comments raising concerns about the longevity of the PETG spring. If it does not last as expected, a revision of the knob design will be needed.</sub>

[![CC BY-NC-SA 4.0][cc-by-nc-sa-shield]][cc-by-nc-sa]

[![CC BY-NC-SA 4.0][cc-by-nc-sa-image]][cc-by-nc-sa]

[cc-by-nc-sa]: http://creativecommons.org/licenses/by-nc-sa/4.0/
[cc-by-nc-sa-image]: https://licensebuttons.net/l/by-nc-sa/4.0/88x31.png
[cc-by-nc-sa-shield]: https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg
