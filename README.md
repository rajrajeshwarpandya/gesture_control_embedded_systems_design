# ESP32 Gesture Glove - BLE AirMouse

An ESP32-based Bluetooth Low Energy (BLE) AirMouse glove using an MPU6050 6-DOF IMU and flex sensors for cursor control, clicks, dragging, and scrolling.

## Features

- **BLE HID Mouse**: Connects wirelessly as `Glove AirMouse` using the `BleMouse` library.
- **IMU Cursor Control**: Tracks hand pitch and yaw via the MPU6050 with deadzone filtering, axis inversion options, and EMA smoothing.
- **Click Stabilization**: Freezes cursor movement briefly during and after clicks (`CLICK_FREEZE_CYCLES`, `DOUBLE_CLICK_FREEZE_CYCLES`) to prevent finger taps from shifting the pointer off target.
- **Gesture Debouncing**: Uses hysteresis on flex sensor resistance and hold/settle timers to avoid false triggers when transitioning between multi-finger poses.
- **Serial Diagnostics & Recalibration**:
  - Send `c` or `r` over Serial (`115200` baud) to recalibrate sensors.
  - Send `d` to toggle debug resistance and gesture state output.

## Gesture Controls

| Gesture | Action | Notes |
| --- | --- | --- |
| Fingers relaxed + move hand | Move cursor | Controlled by MPU6050 gyro |
| Index bent (`> 140 kOhm`) | Left click | Rapid double-tap triggers double click |
| Middle bent (`> 170 kOhm`) | Right click | Supports quick tap (40–150 ms) or hold (`>= 150 ms`) |
| Thumb + Index bent | Double click | Fires two left clicks (`40 ms` apart) |
| Index + Middle bent | Middle mouse drag | Holds `MOUSE_MIDDLE` while moving |
| Thumb + Middle bent + tilt hand | Scroll up / down | Tilt pitch above/below `8.0 deg` threshold |
| Thumb + Index + Middle bent (fist) | Right mouse drag | Holds `FUSION_ORBIT_BUTTON` (`MOUSE_RIGHT` by default) |

## Hardware Connections

### MPU6050 (I2C)
- `SDA` -> `GPIO 21`
- `SCL` -> `GPIO 22`
- `VCC` -> `3.3V`
- `GND` -> `GND`

### Flex Sensors
Voltage divider circuit per sensor: `3.3V -> Flex Sensor -> GPIO -> 68 kOhm -> GND`

| Finger | GPIO | Status | Bend Threshold |
| --- | --- | --- | --- |
| Thumb | `35` | Active | `160 kOhm` |
| Index | `32` | Active | `140 kOhm` |
| Middle | `33` | Active | `170 kOhm` |
| Ring | `25` | Declared (unused) | `130 kOhm` |
| Pinky | `26` | Declared (unused) | `130 kOhm` |

### Status LED
- `GPIO 2`: Blinks while advertising over BLE, stays solid once connected.

## Setup & Usage

1. Install the `ESP32 BLE Mouse` library (`BleMouse.h`) in the Arduino IDE or PlatformIO.
2. Open `final.ino` and select your ESP32 board.
3. Upload the sketch and open the Serial Monitor at `115200` baud.
4. Keep the glove flat on a table during startup while the MPU6050 calibrates.
5. Pair with `Glove AirMouse` in your computer's Bluetooth settings.
