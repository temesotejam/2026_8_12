# CoreS3 / ESP32-S3 virtual CI experiment

This repository is a proof of concept for checking embedded firmware behavior before flashing physical hardware.

It now has a **Wokwi-free path** based on the official Espressif ESP32-S3 QEMU, plus a fast native C++ test. Wokwi remains optional for comparison and for its higher-level CoreS3 board model.

## What is running now

GitHub Actions performs four independent checks on every push/PR:

1. **CoreS3 hardware-topology contract**
   - validates the machine-readable CoreS3 wiring model in `sim/cores3/board_topology.json`,
   - checks Port A/B/C -> GPIO -> M5-Bus relationships,
   - checks all 30 M5-Bus pins,
   - checks LCD/microSD shared SPI wiring,
   - checks internal I2C addresses and camera/audio wiring,
   - requires `CORES3_TOPOLOGY_CHECK:PASS`.

2. **Native shared-logic test**
   - builds a pure C++ test with the host `g++`,
   - executes the same `HeartbeatGate` logic used by the embedded programs,
   - requires `NATIVE_CHECK:PASS`.

3. **Real CoreS3 PlatformIO build**
   - targets `m5stack-cores3`,
   - uses Arduino + M5Unified,
   - initializes the CoreS3 display in `src/main.cpp`,
   - proves the production-side firmware still compiles after shared logic changes.

4. **ESP32-S3 QEMU runtime test — no Wokwi token required**
   - builds an ESP-IDF image for ESP32-S3 with PlatformIO,
   - creates a complete 8 MB SPI flash image containing bootloader, partition table, and application,
   - downloads a pinned official Espressif QEMU binary and verifies its SHA-256,
   - boots the virtual ESP32-S3,
   - lets the real ROM/bootloader/ESP-IDF/FreeRTOS startup execute,
   - requires these runtime messages:

```text
QEMU_BOOT:ESP32S3
QEMU_HEARTBEAT:1
QEMU_HEARTBEAT:2
QEMU_HEARTBEAT:3
QEMU_CHECK:PASS
```

The QEMU serial log is uploaded as a GitHub Actions artifact.

## CoreS3 wiring is now a first-class simulation input

The virtual-board work is no longer based on loose GPIO notes. `sim/cores3/board_topology.json` records the board connection graph used by future peripheral models.

Key external mappings are:

```text
Port A (CoreS3 main unit)
  GND / 5V / GPIO2 SDA / GPIO1 SCL

Port B (Base DIN via M5-Bus)
  GND / 5V / GPIO9 PB_OUT / GPIO8 PB_IN

Port C (Base DIN via M5-Bus)
  GND / 5V / GPIO17 PC_TX / GPIO18 PC_RX
```

The topology also records the full M5-Bus 30-pin map, internal I2C bus, LCD + microSD shared SPI bus, touch, camera, audio, IMU, RTC, and the fact that BMM150 sits behind the BMI270 auxiliary bus rather than directly on the ESP32-S3 I2C bus.

See [`docs/CORES3_HARDWARE_MAP.md`](docs/CORES3_HARDWARE_MAP.md) for the human-readable map.

## The shared logic

`include/smoke_logic.hpp` is deliberately hardware-independent. Both the real CoreS3 program and the QEMU program use the same state logic:

```text
heartbeat 1 -> not passed
heartbeat 2 -> not passed
heartbeat 3 -> PASS (latched)
```

This small example is only a stand-in for the architecture we actually care about. Later, GNSS parsing, health/state machines, timeout handling, navigation math, and other deterministic logic can be moved into shared modules and exercised in exactly the same way.

## Repository layout

```text
.
├── include/
│   └── smoke_logic.hpp
├── src/
│   └── main.cpp
├── test/
│   └── native_smoke.cpp
├── qemu-pio/
│   ├── platformio.ini
│   ├── sdkconfig.defaults
│   └── src/main.cpp
├── sim/
│   └── cores3/
│       └── board_topology.json
├── tools/
│   ├── make_flash_image.py
│   └── validate_cores3_topology.py
├── docs/
│   ├── CORES3_HARDWARE_MAP.md
│   └── TEST_SCOPE.md
├── wokwi.toml
├── diagram.json
└── .github/workflows/ci.yml
```

## Build the real CoreS3 firmware locally

```bash
pio run
```

## Run the fast native test locally

```bash
g++ -std=c++17 -Wall -Wextra -Werror -Iinclude test/native_smoke.cpp -o native-smoke
./native-smoke
```

Expected output:

```text
NATIVE_CHECK:PASS
```

## Validate the board topology locally

```bash
python tools/validate_cores3_topology.py
```

Expected output includes:

```text
CORES3_TOPOLOGY_CHECK:PASS
```

## QEMU path

The CI workflow intentionally installs the QEMU runtime from Espressif's official prebuilt release rather than using a large Docker image. No Wokwi account/token is involved in this job.

```text
PlatformIO / ESP-IDF
        ↓
bootloader.bin + partitions.bin + firmware.bin
        ↓
8 MB virtual SPI flash
        ↓
Espressif QEMU (machine=esp32s3)
        ↓
ESP32-S3 ROM
        ↓
ESP-IDF bootloader
        ↓
FreeRTOS
        ↓
app_main()
        ↓
QEMU_CHECK:PASS
```

## What this does *not* mean yet

The QEMU test is an **ESP32-S3 SoC runtime test**, not yet a complete emulation of every CoreS3 external component. The board topology is now defined and CI-validated, but the individual external device models still need to be attached to it.

The next concrete device targets are:

1. ILI9342C LCD -> 320x240 framebuffer / PNG artifact
2. AW9523B subset used by startup/reset paths
3. AXP2101 subset used by LCD power/backlight and board startup
4. FT6336U virtual touch
5. microSD SPI storage
6. attachable Port A/B/C virtual devices
7. GNSS and onboard sensor models

Physical I2C signal integrity, actual power/noise behavior, RF/GNSS reception, vibration, thermal behavior and similar analog effects still require real hardware/HIL testing.

## Optional Wokwi comparison

The original Wokwi smoke test remains available. If `WOKWI_CLI_TOKEN` is configured as a repository secret, CI also runs the CoreS3 firmware in Wokwi and requires `SIM_CHECK:PASS`. Without the token, this job is skipped and the native/QEMU/topology tests still run normally.
