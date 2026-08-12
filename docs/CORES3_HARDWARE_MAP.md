# CoreS3 hardware topology contract

This document is the wiring contract for the Wokwi-free virtual CoreS3 effort. The machine-readable source used by CI is `sim/cores3/board_topology.json`.

The goal is not merely to know individual GPIO numbers. The simulator needs to know **which physical connector, M5-Bus pin, shared bus, expander pin, and peripheral each signal reaches** so future virtual devices can be connected the same way as the real CoreS3.

## External HY2.0-4P ports

| Port | Physical exposure | Black | Red | Yellow | White | Intended role |
|---|---|---|---|---|---|---|
| A | CoreS3 main unit | GND | 5V | GPIO2 / SDA | GPIO1 / SCL | I2C |
| B | Base DIN via M5-Bus | GND | 5V | GPIO9 / PB_OUT | GPIO8 / PB_IN | General I/O |
| C | Base DIN via M5-Bus | GND | 5V | GPIO17 / PC_TX | GPIO18 / PC_RX | UART |

Port A is directly visible on the CoreS3 main-board schematic as the GH2.0-4P connector connected to `BUS_PA_SCL`, `BUS_PA_SDA`, `BUS_OUT`, and GND. The CoreS3 + Base DIN stack compatibility map routes Port B through M5-Bus pins 4/10 and Port C through pins 15/16.

For the virtual board this means a future test can attach a device to a logical connector instead of hard-coding an arbitrary GPIO. For example, a virtual Unit connected to `PORT.A` will automatically receive GPIO2/GPIO1 as SDA/SCL.

## M5-Bus 30-pin map

| Pin | CoreS3 signal |
|---:|---|
| 1 | GND |
| 2 | GPIO10 / ADC |
| 3 | GND |
| 4 | GPIO8 / PB_IN |
| 5 | GND |
| 6 | RST / EN |
| 7 | GPIO37 / SPI MOSI |
| 8 | GPIO5 / GPIO |
| 9 | GPIO35 / SPI MISO |
| 10 | GPIO9 / PB_OUT |
| 11 | GPIO36 / SPI SCK |
| 12 | 3V3 |
| 13 | GPIO44 / RXD0 |
| 14 | GPIO43 / TXD0 |
| 15 | GPIO18 / PC_RX |
| 16 | GPIO17 / PC_TX |
| 17 | GPIO12 / Internal I2C SDA |
| 18 | GPIO11 / Internal I2C SCL |
| 19 | GPIO2 / Port A SDA |
| 20 | GPIO1 / Port A SCL |
| 21 | GPIO6 / GPIO |
| 22 | GPIO7 / GPIO |
| 23 | GPIO13 / I2S DOUT |
| 24 | GPIO0 / I2S LRCK |
| 25 | HPWR / HVIN from Base DIN |
| 26 | GPIO14 / I2S DIN |
| 27 | HPWR / HVIN from Base DIN |
| 28 | 5V |
| 29 | HPWR / HVIN from Base DIN |
| 30 | BAT |

This table is important for future Module/Base simulation: a virtual M5Stack module should attach to M5-Bus pin numbers, not directly to an assumed ESP32 GPIO map.

## LCD + microSD shared SPI topology

The LCD is an ILI9342C, 320x240. The LCD and microSD share part of the same ESP32-S3 SPI wiring:

```text
ESP32-S3
  GPIO37 MOSI ----------------+---- ILI9342C MOSI
                              +---- microSD MOSI
  GPIO36 SCK -----------------+---- ILI9342C SCK
                              +---- microSD SCK
  GPIO35 -------------------------- ILI9342C DC
                              +---- microSD MISO
  GPIO3  -------------------------- ILI9342C CS
  GPIO4  -------------------------- microSD CS

AW9523B P1_1 ---------------------- ILI9342C RESET
AXP2101 DLDO1 --------------------- LCD backlight
AXP2101 LX1 ----------------------- LCD power
```

GPIO35 having two board-level roles is deliberate: the LCD does not use an SPI MISO line, so that ESP32-S3 pin is used as LCD D/C while also serving as SD MISO. A future emulator must model this topology rather than treating the LCD and SD as independent arbitrary SPI buses.

## Internal I2C bus

The main internal I2C bus is GPIO12=SDA and GPIO11=SCL.

| Device | Address | Notes |
|---|---:|---|
| AXP2101 | 0x34 | Power management |
| AW9523B | 0x58 | I/O expander and reset/interrupt routing |
| BM8563 | 0x51 | RTC |
| BMI270 | 0x69 | Accelerometer + gyro |
| ES7210 | 0x40 | Microphone codec |
| AW88298 | 0x36 | Speaker amplifier |
| FT6336U | 0x38 | Capacitive touch |
| GC0308 | 0x21 | Camera control/SCCB |
| LTR-553ALS-WA | 0x23 | Proximity sensor |

The BMM150 magnetometer is different. It is at address 0x10 but is connected behind the BMI270 auxiliary/sensor-hub bus, not directly as another ESP32-S3 internal-I2C device. The virtual model records this explicitly because pretending it is directly visible on GPIO12/11 would hide a real hardware integration detail.

## AW9523B-routed controls already captured

The topology records the controls needed for early peripheral emulation:

- `P0_0` -> touch reset
- `P0_2` -> AW88298 reset
- `P1_0` -> camera reset
- `P1_1` -> LCD reset
- `P1_2` -> touch interrupt
- `P1_3` -> AW88298 interrupt

The schematic also contains additional power-path/control signals through this expander. Those will be modeled when power/USB behavior becomes relevant rather than faking their behavior early.

## Camera wiring

The GC0308 control path shares the internal I2C/SCCB pins (GPIO12/GPIO11). Pixel/timing signals are:

| Camera signal | ESP32-S3 GPIO |
|---|---:|
| PCLK | 45 |
| VSYNC | 46 |
| HREF | 38 |
| D0 | 39 |
| D1 | 40 |
| D2 | 41 |
| D3 | 42 |
| D4 | 15 |
| D5 | 16 |
| D6 | 48 |
| D7 | 47 |

A future virtual camera can therefore inject a deterministic test frame through the same pin topology instead of bypassing the camera driver.

## Touch wiring

FT6336U uses the internal I2C bus at 0x38. Reset and interrupt are routed through the AW9523B expander. This makes touch a good second-stage UI test after the LCD framebuffer is working: a CI scenario can set a virtual touch coordinate and compare the resulting 320x240 framebuffer.

## Audio wiring

| Signal | GPIO |
|---|---:|
| I2S BCK | 34 |
| I2S WCK | 33 |
| Speaker DOUT | 13 |
| Microphone DIN | 14 |
| MCLK | 0 |

The ES7210 codec is on internal I2C address 0x40 and the AW88298 amplifier is at 0x36.

## What is already enforced in CI

`tools/validate_cores3_topology.py` checks that the machine-readable map contains the expected:

- Port A/B/C GPIO and M5-Bus relationships,
- all 30 M5-Bus positions,
- LCD/SD shared SPI mapping,
- internal I2C pins and addresses,
- BMM150 auxiliary-bus placement,
- camera data/timing pins,
- audio I2S pins.

If a future edit accidentally changes one of these board contracts, GitHub Actions fails before running the firmware simulation.

## Emulation order from here

The topology is now defined first, so actual virtual devices can plug into known connection points. The planned order is:

1. ILI9342C SPI command/pixel model -> `cores3-screen.png`
2. AW9523B subset needed by M5Unified startup/reset paths
3. AXP2101 subset needed by LCD power/backlight and board startup
4. FT6336U touch injection
5. microSD SPI storage
6. Port A/B/C external-device attachment API
7. virtual GNSS on a selected external UART path / M5-Bus module path
8. BMI270/BMM150 and the remaining onboard devices

The intended end state is a virtual CoreS3 where code talks to the same GPIO/bus topology as the physical board and CI can inspect serial output, screen pixels, external-port traffic, and virtual sensor behavior.

## Authoritative references

- M5Stack CoreS3 product/pin map: https://docs.m5stack.com/en/core/CoreS3
- M5Stack CoreS3 schematic: https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/490/Sch_M5_CoreS3_v1.0.pdf
- CoreS3 + Base DIN stack compatibility: https://docs.m5stack.com/en/compatible_stack?base=M132&host=K128
