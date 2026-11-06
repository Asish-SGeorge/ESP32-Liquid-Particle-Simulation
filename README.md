# ESP32 Liquid Particle Simulation

A two-container liquid particle simulation built with an ESP32, MPU6500, MAX7219 LED matrix, and an Android companion app.

Tilt either container left or right to move the liquid. Particles can pour from the top corners of one container and enter the other through its top-middle port. This repository contains the ESP32 firmware and the complete Android Studio source project.

## Repository contents

- `firmware/ESP32_Portrait_Glass_Liquid_RoundTrip.ino` — standalone ESP32 firmware.
- `android/` — Android Studio project source, including the Kotlin activity, manifest, Gradle configuration, project README, and companion sketch.

## Hardware

- ESP32 DevKit V1
- MPU6500 accelerometer/gyroscope module
- MAX7219 FC-16 4-in-1 8×32 LED matrix
- Android phone

### Wiring

| Device | Signal | ESP32 GPIO |
|---|---|---:|
| MAX7219 | DIN | 23 |
| MAX7219 | CLK | 18 |
| MAX7219 | CS / LOAD | 5 |
| MPU6500 | SDA | 21 |
| MPU6500 | SCL | 22 |
| MPU6500 | AD0 | GND (I²C address 0x68) |

Use a regulated 5 V supply for the LED matrix and connect its ground to ESP32 ground. Power the MPU6500 from 3.3 V and share ground.

## Getting started

1. Open the standalone `.ino` file in Arduino IDE.
2. Install the `MD_MAX72XX` library and upload to an ESP32 DevKit V1.
3. Connect the Android phone to the ESP32 Wi-Fi access point: `ESP32_LIQUID` (password: `liquid123`).
4. Open the `android/` folder in Android Studio.
5. Build the APK in Android Studio, install it, enable motion, and calibrate the phone upright.

The ESP32 web page is available at `http://192.168.4.1` while connected to its access point.

## Direction settings

The current sensor/display mounting uses `ONE_AXIS_DIRECTION = -1.0f` for the matrix and `PHONE_TILT_DIRECTION = -1.0` for the Android phone. Change only the relevant setting if one device's left/right response is reversed.

The LED matrix is configured as a portrait 8-wide × 32-high glass. Its physical vertical pixel mapping is reversed in `renderLiquid()` so gravity appears downward with the FC-16 board mounted clockwise.

## Verification note

The repository contains source code, not a prebuilt APK. An Android APK build and physical hardware test have not been performed as part of this upload. Verify display orientation and tilt direction on the actual assembly before final use.

## License

No license is specified. Unless a license is added, standard copyright restrictions apply to reuse and redistribution.
