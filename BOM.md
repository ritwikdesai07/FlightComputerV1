# Flight Computer PCB BOM (Arduino Uno Compatible)

This bill of materials assumes you are using a full Arduino Uno as the MCU and building
a shield that keeps the current sketch unchanged (ICM-20948 IMU, BMP280 barometer, and
microSD logging over SPI with CS on D10).【F:flightComputer1.ino†L1-L90】

## Core ICs & Modules
| Ref | Qty | Part | Example MPN / Notes | Why |
| --- | --- | ---- | ------------------- | --- |
| U2 | 1 | ICM-20948 IMU | TDK InvenSense ICM-20948 (JLCPCB C726001) | Matches `ICM_20948` library and I2C initialization in code.【F:flightComputer1.ino†L1-L36】【F:flightComputer1.ino†L96-L120】 |
| U3 | 1 | BMP280 Barometer | Bosch BMP280 (JLCPCB C83291) | Matches `Adafruit_BMP280` library and default I2C init.【F:flightComputer1.ino†L1-L6】【F:flightComputer1.ino†L79-L111】 |
| J1 | 1 | microSD socket | Hirose DM3AT-SF-PEJM5 (JLCPCB C114218, no push-push) | Required for SD logging with CS on D10.【F:flightComputer1.ino†L54-L90】 |

## Power & Level Shifting (5V MCU to 3.3V Sensors)
| Ref | Qty | Part | Example MPN / Notes | Why |
| --- | --- | ---- | ------------------- | --- |
| U4 | 1 | 3.3V LDO regulator | MCP1700-3302E/TO, AMS1117-3.3 | Powers BMP280, ICM-20948, and microSD at 3.3V. |
| U5 | 1 | I2C level shifter | BSS138-based bidirectional level shifter | I2C runs at 3.3V for BMP280/ICM-20948 while MCU is 5V. |
| U6 | 1 | SPI level shifter (3.3V) | 74LVC125/74AHC125 | Shifts 5V SPI (SCK/MOSI/CS) down to 3.3V for microSD. |
| R? | 3–4 | Pull-up resistors | 4.7k–10k to 3.3V | I2C pull-ups on SDA/SCL at 3.3V. |

## Clock & Reset
Omitted since the Arduino Uno provides the MCU clock and reset circuitry on the base board.

## Connectors & I/O
| Ref | Qty | Part | Example MPN / Notes | Why |
| --- | --- | ---- | ------------------- | --- |
| J2 | 1 | Servo header | 3-pin (GND/5V/SIG) | Servo on D9 as in sketch.【F:flightComputer1.ino†L15-L36】 |
| J3 | 1 | USB or FTDI header | 6-pin FTDI or USB-serial | For programming/debugging if not using Arduino Uno board. |
| J4 | 1 | Battery input connector | JST-PH-2 or screw terminal | Power input to 5V regulator or 5V rail. |

## Passive Support
| Ref | Qty | Part | Notes |
| --- | --- | ---- | ----- |
| C? | 4–6 | 0.1 µF decoupling capacitors | One per IC power pin (IMU, barometer, level shifters). |
| C? | 1–2 | Bulk capacitors | 10 µF–47 µF on 5V and 3.3V rails. |

## Notes / Integration Tips
- Keep I2C devices (ICM-20948 and BMP280) on the same 3.3V I2C bus through a level shifter.
- Keep SD card SPI on 3.3V; shift SCK/MOSI/CS down; MISO can be read at 3.3V by the ATmega328P.
- Use D10 for SD card chip-select to keep the sketch unchanged.【F:flightComputer1.ino†L54-L60】
- Servo signal is on D9; provide a stable 5V rail for the servo (consider separate regulator if needed).【F:flightComputer1.ino†L15-L36】
