# ESP32 Portrait Glass Liquid — Android companion

This project pairs a portrait-mounted MAX7219 LED matrix with an Android phone. Each device behaves like a glass container controlled by one left/right tilt axis. Tilt drives the liquid simulation, and particles can transfer between the two containers through the top-corner outlets and top-middle inlet ports.

## Hardware

- ESP32 DevKit V1
- MPU6500 accelerometer/gyroscope module
- MAX7219 FC-16 4-in-1 8×32 LED matrix
- Android phone running the companion app

### Wiring

| Module | Signal | ESP32 pin |
|---|---|---:|
| MAX7219 | DIN | GPIO 23 |
| MAX7219 | CLK | GPIO 18 |
| MAX7219 | CS / LOAD | GPIO 5 |
| MPU6500 | SDA | GPIO 21 |
| MPU6500 | SCL | GPIO 22 |
| MPU6500 | AD0 | GND (I²C address 0x68) |

Power the MAX7219 matrix from a regulated 5 V supply. Connect its GND to ESP32 GND. Power the MPU6500 from 3.3 V and share ground. Mount the MPU6500 parallel to the front plane of the matrix so its X/Y accelerometer axes lie in the matrix plane.

Rotate the complete 4-in-1 FC-16 board clockwise by 90° so its 32-pixel dimension is vertical. Keep the opening at the top and hold the matrix still during ESP32 startup calibration. The sketch maps the logical 8-wide × 32-high glass to this portrait orientation and reverses the physical vertical pixel index so gravity appears downward on the mounted display.

## Features

- One-axis left/right tilt control for both containers.
- Gravity-based liquid particle simulation on the LED matrix and Android screen.
- Pouring through the matching top-left or top-right outlet after the tilt threshold is crossed.
- Top-middle inlet for particles arriving from the other container.
- Stable particle IDs and duplicate-transfer protection to avoid creating extra particles during HTTP retries.
- Native Android motion bridge, so the app does not depend on browser motion permission over the ESP32's local HTTP hotspot.

## ESP32 firmware

Open `ESP32_Portrait_Glass_Liquid_RoundTrip.ino` in Arduino IDE. Install the **MD_MAX72XX** library, select **ESP32 Dev Module / ESP32 DevKit V1**, choose the correct serial port, and upload.

The ESP32 creates this Wi-Fi access point:

- SSID: `ESP32_LIQUID`
- Password: `liquid123`
- Device page: `http://192.168.4.1`

Connect the Android phone to this network. If Android warns that the network has no internet, choose **Stay connected**.

## Android app

Open this folder in Android Studio. The app uses Android SDK 35 and Java 17. Allow Gradle to sync, then select **Build → Build Bundle(s) / APK(s) → Build APK(s)**. Install `app/build/outputs/apk/debug/app-debug.apk` on the phone.

Launch **ESP32 Phone Liquid**, enable motion, and calibrate the phone while it is upright in portrait orientation. Keep the phone connected to the ESP32 access point.

## Direction and calibration

Both direction signs are currently inverted for the current sensor/display mounting:

- `ONE_AXIS_DIRECTION = -1.0f` controls ESP32 matrix tilt direction.
- `PHONE_TILT_DIRECTION = -1.0` controls Android phone tilt direction in the embedded page JavaScript.

If only one device responds in the wrong direction, change only that device's sign. Keep the vessel upright and still during calibration. The pour threshold is controlled by `POUR_GRAVITY_THRESHOLD` in the firmware.

## Project layout

- `ESP32_Portrait_Glass_Liquid_RoundTrip.ino` — ESP32 access point, matrix simulation, web server, and particle-transfer logic.
- `ESP32_Phone_Liquid_Android_Companion.ino` — companion copy of the ESP32-served simulation source included with the Android project.
- `app/` — Android application source, manifest, and Gradle configuration.
- `build.gradle.kts`, `settings.gradle.kts`, `gradle.properties` — Android project build configuration.

## Verification note

The source files are packaged for use in Arduino IDE and Android Studio. An APK build and a physical hardware test have not been performed as part of this repository upload. Verify tilt direction and display orientation on the actual assembly before final use.
