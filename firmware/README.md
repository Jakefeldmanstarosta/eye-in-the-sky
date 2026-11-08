# ESP-Drone firmware setup

Firmware: [espressif/esp-drone](https://github.com/espressif/esp-drone) (GPL-3.0), included as a git submodule at `firmware/esp-drone` (pinned to commit `9d5a4f8`).

Upstream docs: https://docs.espressif.com/projects/espressif-esp-drone/en/latest/gettingstarted.html
Phone apps: [Android](https://github.com/EspressifApps/ESP-Drone-Android), [iOS](https://github.com/EspressifApps/ESP-Drone-iOS). Desktop: [cfclient fork](https://github.com/leeebo/crazyflie-clients-python).

## 1. Get the code

Fresh clone:

```
git clone --recursive https://github.com/Jakefeldmanstarosta/eye-in-the-sky.git
```

Already cloned (the `firmware/esp-drone` folder is empty):

```
git submodule update --init --recursive
```

## 2. Install ESP-IDF (macOS)

ESP-Drone supports ESP-IDF **release/v5.5** (or v6.1). Use v5.5.

```
xcode-select --install
brew install cmake ninja dfu-util python3
mkdir -p ~/esp && cd ~/esp
git clone -b release/v5.5 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf && ./install.sh esp32
. ./export.sh        # run this in every new terminal
```

Guide: https://docs.espressif.com/projects/esp-idf/en/release-v5.5/esp32/get-started/index.html

## 3. Wiring (ESP-Drone's standard pins for classic ESP32)

We use ESP-Drone's default pin map for the classic ESP32 target (`esplane with esp32 onboard`), so **no pin changes are needed in menuconfig**. Wire the breadboard to match:

| Function | ESP32 GPIO | Firmware name |
|---|---|---|
| Front left motor (M1) | **4** | `MOTOR01_PIN` |
| Back left motor (M2) | **33** | `MOTOR02_PIN` |
| Back right motor (M3) | **32** | `MOTOR03_PIN` |
| Front right motor (M4) | **25** | `MOTOR04_PIN` |
| MPU-6050 SDA | **21** | `I2C0_PIN_SDA` |
| MPU-6050 SCL | **22** | `I2C0_PIN_SCL` |
| MPU-6050 INT (optional) | 19 | `MPU_PIN_INT` |
| Buzzer +/- (optional) | 27 / 26 | `BUZ1_PIN_POS` / `BUZ2_PIN_NEG` |
| LEDs blue / green / red (optional) | 18 / 5 / 23 | `LED_PIN_*` |

Notes:
- The M1..M4 corners follow the Crazyflie quad-X layout: M1 front left, M2 back left, M3 back right, M4 front right. M1 and M3 spin counter-clockwise, M2 and M4 clockwise. The "first run" checks below confirm this on the real drone.
- Each motor pin drives a MOSFET/driver input, not the motor directly.
- GPIO 4, 25, 32 and 33 are safe at boot. The old wiring (motors on 12/13/14/26 with SDA/SCL swapped on 21/22) is **not** used any more. GPIO12 is a boot-strapping pin and was risky anyway.

## 4. Configure and build

```
cd firmware/esp-drone
idf.py set-target esp32
idf.py menuconfig
```

In menuconfig:

1. **ESP_Drone hardware version** -> `esplane with esp32 onboard (old version)` (the only profile for classic ESP32, and the default).
2. Leave the pin settings at their defaults (table above).
3. **Motor type** is brushed (715/720), i.e. PWM to a MOSFET/driver per brushed motor.
4. Optional: turn off the buzzer (`BUZZER_ON`) if you don't have one.

Build and flash (replace the port):

```
idf.py -p /dev/cu.usbserial-XXXX build flash monitor
```

Find the port with `ls /dev/cu.*`.

## 5. First run

1. **Remove the propellers.**
2. The board starts a Wi-Fi access point: SSID `ESP-DRONE_xxxx`, password `12345678`.
3. Connect your phone to it and open the ESP-Drone app.
4. With low throttle (props still off), check which motor spins for each corner and its direction. The app may also show per-motor outputs (M1..M4). Fix the `MOTORxx_PIN` mapping until the corners and rotation directions match the standard quad-X layout.
5. Put the drone flat and level, then use the app's Roll/Pitch trim (or `PITCH_CALIB`/`ROLL_CALIB` in menuconfig) so it reads level.
6. Only then fit propellers and test with the drone restrained.

## Troubleshooting

- **IMU not found:** check SDA is on GPIO21 and SCL on GPIO22 (not swapped), check wiring and power (3.3 V), keep jumper wires short.
- **Boot loop or no boot:** disconnect the motor drivers and sensor from the board and retry, then reconnect one at a time.
- **No Wi-Fi network:** check the serial monitor output, and make sure `TRANSPORT_NATIVE_WIFI` is selected.
