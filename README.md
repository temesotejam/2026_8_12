# CoreS3 / ESP32-S3 virtual CI experiment

This repository is a proof of concept for checking embedded firmware behavior before flashing physical hardware.

It now has a **Wokwi-free path** based on the official Espressif ESP32-S3 QEMU, plus a fast native C++ test. Wokwi remains optional for comparison and for its higher-level CoreS3 board model.

## What is running now

GitHub Actions performs three independent checks on every push/PR:

1. **Native shared-logic test**
   - builds a pure C++ test with the host `g++`,
   - executes the same `HeartbeatGate` logic used by the embedded programs,
   - requires `NATIVE_CHECK:PASS`.

2. **Real CoreS3 PlatformIO build**
   - targets `m5stack-cores3`,
   - uses Arduino + M5Unified,
   - initializes the CoreS3 display in `src/main.cpp`,
   - proves the production-side firmware still compiles after shared logic changes.

3. **ESP32-S3 QEMU runtime test — no Wokwi token required**
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

## The shared part

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
│   └── smoke_logic.hpp       # hardware-independent logic shared by all tests
├── src/
│   └── main.cpp              # real M5Stack CoreS3 / M5Unified program
├── test/
│   └── native_smoke.cpp      # fast host-side test
├── qemu-pio/
│   ├── platformio.ini        # ESP32-S3 + ESP-IDF QEMU build
│   ├── sdkconfig.defaults    # 8 MB flash configuration
│   └── src/main.cpp          # QEMU runtime harness using the shared logic
├── tools/
│   └── make_flash_image.py   # assembles bootloader/partitions/app into SPI flash
├── wokwi.toml                # optional Wokwi path
├── diagram.json              # optional Wokwi CoreS3 board
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

## QEMU path

The CI workflow intentionally installs the QEMU runtime from Espressif's official prebuilt release rather than using a large Docker image. No Wokwi account/token is involved in this job.

The runtime path is:

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

## What this does *not* mean

The QEMU test is an **ESP32-S3 SoC runtime test**, not a complete emulation of every CoreS3 external component. The real CoreS3 program is currently build-tested, while the QEMU runtime harness shares the hardware-independent application logic with it.

LCD/touch/audio/camera, physical I2C signal quality, power behavior, RF/GNSS reception, sensor noise, vibration, and similar hardware effects still require either more virtual peripheral models or real hardware/HIL testing. See `docs/TEST_SCOPE.md`.

## Optional Wokwi comparison

The original Wokwi smoke test remains available. If `WOKWI_CLI_TOKEN` is configured as a repository secret, CI also runs the CoreS3 firmware in Wokwi and requires `SIM_CHECK:PASS`. Without the token, this job is skipped and the native/QEMU tests still run normally.

## Next useful extension

A good next experiment is a **virtual UART GNSS source**. That would let CI inject deterministic NMEA/UBX data, dropouts, corrupt packets, delays, and recovery sequences and then judge the firmware's real parser/state behavior automatically.
