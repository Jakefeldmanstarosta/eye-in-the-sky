# Eye in the Sky – flight electronics wiring (KiCad)

Open `eye-in-the-sky-wiring.kicad_pro` in KiCad 8 or newer (made with KiCad 10.0.7). The schematic is `eye-in-the-sky-wiring.kicad_sch`. The custom symbols are in `EyeInTheSky.kicad_sym`, which `sym-lib-table` registers for this project. Wires are joined by **net labels**: two pins with the same label are connected. ERC passes with 0 errors and 0 warnings.

## Parts

| Ref | Part | Pin labels |
|---|---|---|
| U1 | ESP32 DevKit V1, 30-pin (DOIT layout: ESP32-WROOM-32, CP2102, AMS1117) | Same as the board's silkscreen, in physical order from the antenna end down: left `EN VP VN D34 D35 D32 D33 D25 D26 D27 D14 D12 D13 GND VIN`, right `D23 D22 TX0 RX0 D21 D19 D18 D5 TX2 RX2 D4 D2 D15 GND 3V3` |
| U2 | Jetson Nano Developer Kit 4 GB, J41 40-pin header | NVIDIA J41 signal names, odd pins on the left and even on the right, as on the header |
| M1–M4 | Motor leads (V, SIG, GND) | — |
| U3 | MPU-6050 (GY-521) | Module silkscreen |
| U4 | BMP390 breakout (Adafruit 4816 pinout) | Module silkscreen |
| U5 | INA226 module with 0.1 Ω shunt | Module silkscreen |
| U6 | 5 V buck / BEC, ≥ 5 A | — |
| J1 | LiPo battery (XT60) | — |
| R1 | 10 kΩ pull-up for the INA226 alert line | — |

## ESP32 pin map

| Silkscreen | GPIO | Net | Goes to |
|---|---|---|---|
| D4 | 4 | MOTOR_M1 | Front-left motor SIG |
| D33 | 33 | MOTOR_M2 | Back-left motor SIG |
| D32 | 32 | MOTOR_M3 | Back-right motor SIG |
| D25 | 25 | MOTOR_M4 | Front-right motor SIG |
| D21 | 21 | I2C0_SDA | MPU-6050 SDA, BMP390 SDI, INA226 SDA |
| D22 | 22 | I2C0_SCL | MPU-6050 SCL, BMP390 SCK, INA226 SCL |
| D19 | 19 | MPU_INT | MPU-6050 INT |
| VN | 39 | BARO_INT | BMP390 INT |
| D34 | 34 | PWR_ALERT | INA226 ALE (+ R1 to 3V3) |
| D27 | 27 | UART_ESP_TO_JETSON | Jetson J41 pin 10 (UART_2_RX) |
| D26 | 26 | UART_JETSON_TO_ESP | Jetson J41 pin 8 (UART_2_TX) |
| VIN | – | +5V | Buck output (feeds the board's AMS1117 3.3 V regulator) |
| 3V3 | – | +3V3 | Supplies all sensors |
| GND ×2 | – | GND | Common ground |

All other ESP32 pins are marked no-connect.

## Reasoning

**Motors (GPIO 4 / 33 / 32 / 25).** These match `firmware/README.md` and the ESP-Drone defaults (`MOTOR01..04_PIN`), so no firmware changes are needed. They are not boot-strapping pins, so the motors can't stop the ESP32 from booting. Each motor's **V** wire goes to **VBAT** (the battery side after the INA226 shunt), never to the ESP32. The ESP32 can only supply logic-level current. **SIG** is 3.3 V PWM into the motor's driver or ESC. All motor grounds return to the common GND, so the PWM signal has a reference.

**One I2C bus for every sensor (GPIO 21 / 22).** The firmware already runs the MPU-6050 on I2C0 at 21/22. The BMP390 and INA226 share that bus because their addresses don't clash: MPU-6050 **0x68** (AD0 → GND), BMP390 **0x77** (SDO → 3V3), INA226 **0x40** (A0 = A1 = GND on the module). Sharing the bus means fewer wires and no extra pins. GPIO 16/17 (silkscreen RX2/TX2) stay free for ESP-Drone's second I2C bus (I2C1, used by expansion decks such as a flow or ToF sensor). The breakout boards already have I2C pull-ups, so none are added. If the bus is unreliable with all three connected, remove the pull-ups on two of the boards.

**BMP390 wiring.** CS is tied to 3V3 so the chip starts in I2C mode instead of SPI. SDO is tied to 3V3 to fix the address at 0x77. INT goes to GPIO 39 (silkscreen **VN**). It is an input-only pin, which is fine for an interrupt, and using it saves an output-capable pin. The BMP390 INT output is push-pull, so it needs no pull-up.

**INA226 placement.** The shunt sits in series with the battery **high side**: BAT+ → IN+ → shunt → IN− → VBAT. That way it measures the current for the whole drone (motors, ESP32 and Jetson). VBS is tied to VBAT, so the chip also reads pack voltage and can report power. It handles up to 36 V, which covers 1S–6S. ALE is open-drain, so it needs R1 (10 k to 3V3). It goes to GPIO 34, another input-only pin. You can set an over-current or under-voltage alert threshold and get an interrupt without polling.

**Jetson ↔ ESP32 link: UART.** Jetson J41 pin 8 (UART_2_TX) goes to ESP32 GPIO 26, and J41 pin 10 (UART_2_RX) comes from GPIO 27, with TX/RX crossed. Both sides use 3.3 V logic, so no level shifter is needed. Reasons for a UART:
- It is a simple, wired, low-latency way for the vision stack (`quad_avoid`, Depth-Anything) to send setpoints. Wi-Fi to the ESP-Drone access point would be the alternative.
- UART0 (TX0/RX0 pins) is avoided because it is the ESP32's USB flash/debug port.
- GPIO 26/27 are ESP-Drone's *optional* buzzer pins. Set `BUZZER_ON = n` in menuconfig and remap UART2 to them through the GPIO matrix. The only other free output-capable pins are boot straps (0, 2, 12, 15) or are taken by I2C1, LEDs or SPI.
- On the Jetson, J41 pins 8/10 are `/dev/ttyTHS1`. Run `sudo systemctl disable nvgetty` so a login console doesn't take over the port.

The Jetson's own I2C buses are left unconnected on purpose. The ESP32 is the only I2C master for the flight sensors, so the two boards never fight over the bus.

**Power.**
- VBAT feeds a 5 V buck (≥ 5 A). That buck powers the Jetson through **J41 pins 2 and 4** (both 5 V pins, which shares the current) and the ESP32 through its **VIN** pin.
- The ESP32's on-board 3.3 V regulator supplies the sensors, which draw a few mA in total.
- The Jetson's 3.3 V pins (1, 17) are left open, because tying two regulators' outputs together is unsafe.
- All grounds are tied together. The schematic ties every Jetson GND pin, but in practice two or three wires are enough.
- Alternative: power the Jetson from its barrel jack J25 with jumper J48 fitted, and leave pins 2/4 unconnected. The barrel jack path has input protection that pins 2/4 skip.

## Caveats

- **Battery vs. motors.** The Jetson needs 5 V at up to 4 A, so the buck needs at least about 7 V in (2S or more). Brushed 715/720 motors are 1S motors. If you keep brushed motors on 2S+, limit the PWM duty in firmware or use motors rated for that voltage. Brushless motors with ESCs would also fit this wiring (V/SIG/GND match an ESC lead), but then ESP-Drone has to output ESC-style PWM.
- **USB and 5 V together.** Don't power the ESP32 from USB and the 5 V rail at the same time. Unplug one before connecting the other.
- **Footprints.** None are assigned. This is a wiring/connection schematic for breadboarding, not ready for PCB layout.
