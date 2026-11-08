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

## 3. Our hardware (classic ESP32 dev board on a breadboard)

| Function | ESP32 GPIO |
|---|---|
| Front left motor | 12 |
| Front right motor | 13 |
| Back left motor | 14 |
| Back right motor | 26 |
| MPU-6050 SCL | 21 |
| MPU-6050 SDA | 22 |

Notes:
- **I2C is swapped from the ESP32 default** (default is SDA 21, SCL 22), so SDA/SCL must be set in menuconfig.
- **GPIO12 is a boot-strapping pin.** If it is held high at power-up, a WROOM-32 can fail to boot. If it does, move that motor to GPIO 25, 27, 32 or 33.
- ESP-Drone's own defaults for the ESP32 target are motors on 4/33/32/25 and I2C SDA 21/SCL 22, so every pin below needs changing.

## 4. Configure and build

```
cd firmware/esp-drone
idf.py set-target esp32
idf.py menuconfig
```

In menuconfig:

1. **ESP_Drone hardware version** -> `esplane with esp32 onboard (old version)` (this is the only profile for classic ESP32).
2. Under the I2C pin settings, set `I2C0_PIN_SDA` = **22** and `I2C0_PIN_SCL` = **21**.
3. Under **motors config**, set the motor pins (`MOTOR01_PIN` .. `MOTOR04_PIN`). Upstream does not document which of M1..M4 is which corner, so confirm with the test below before choosing. A starting point is 12 / 13 / 14 / 26 in the order of the table above.
4. **Motor type** is brushed (715/720), i.e. PWM to a MOSFET/driver per brushed motor.
5. Optional: turn off the buzzer (`BUZZER_ON`) if you don't have one. Its default pins on ESP32 are 27 and 26, and 26 clashes with the back right motor.

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

- **IMU not found:** check SDA/SCL aren't swapped in menuconfig, check wiring and power (3.3 V), keep jumper wires short.
- **Boot loop or no boot:** unplug the motor driver from GPIO12 and retry (see strapping pin note).
- **No Wi-Fi network:** check the serial monitor output, and make sure `TRANSPORT_NATIVE_WIFI` is selected.
