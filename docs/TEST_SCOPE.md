# Test scope

This repository deliberately separates three different questions instead of calling every green check a full hardware simulation.

## 1. Native shared-logic test

A passing native test gives strong evidence that deterministic, hardware-independent C++ logic behaves as specified. It is ideal for state machines, parsers, timeout logic, navigation math, filtering, validation rules, and large scenario sets.

It does **not** exercise the ESP32-S3 CPU, FreeRTOS, flash boot, Arduino, M5Unified, or hardware registers.

## 2. ESP32-S3 QEMU runtime test

A passing QEMU test currently proves that:

- an ESP32-S3 ESP-IDF firmware image can be built,
- the virtual SPI flash layout is bootable,
- the ESP32-S3 ROM starts,
- the ESP-IDF second-stage bootloader loads the application,
- ESP-IDF initializes on the emulated SoC,
- FreeRTOS reaches `app_main()`,
- the shared application logic executes over time,
- serial output is observable by CI,
- GitHub Actions can fail the job if expected runtime behavior is absent.

This is substantially more than a compile-only test.

It does **not** currently prove that M5Unified's CoreS3-specific LCD/touch/audio/camera code works in QEMU. The QEMU program is an ESP-IDF harness that reuses the same hardware-independent logic as the CoreS3 firmware.

## 3. Real CoreS3 PlatformIO build

A passing CoreS3 build proves that the real Arduino/M5Unified application remains compatible with the shared code and produces firmware for the `m5stack-cores3` target.

Without a physical board or a higher-level CoreS3 simulator, build success alone does not prove the CoreS3 external hardware path at runtime.

## Optional Wokwi test

Wokwi remains an optional fourth check. When `WOKWI_CLI_TOKEN` is configured, it can run the CoreS3-oriented firmware in Wokwi's CoreS3 model. The self-hosted native and QEMU jobs do not require this token.

## What no current virtual test proves

The current suite does not prove:

- power sequencing, brownouts, or regulator behavior,
- electrical I2C/SPI/UART signal integrity,
- pull-up resistor correctness,
- EMI or motor noise behavior,
- physical timing limits caused by wiring/loads,
- real display/touch/camera/speaker behavior,
- GNSS RF reception or antenna performance,
- real sensor bias, vibration, temperature drift, or magnetic interference.

Those require physical tests or HIL.

## Recommended extension path

The useful goal is not to recreate every CoreS3 transistor. It is to model the interfaces that can expose firmware bugs.

1. Move deterministic application behavior into small shared modules.
2. Test those modules quickly on the host.
3. Execute representative cases inside ESP32-S3 QEMU to include boot/FreeRTOS/SoC behavior.
4. Add a virtual UART GNSS endpoint and exercise real NMEA/UBX parsing.
5. Inject GNSS dropout, corruption, delay, stale data, jumps, and recovery.
6. Add virtual sensor inputs at the narrowest useful interface (first measurement-level, later register/I2C-level if needed).
7. Add a physical CoreS3 HIL runner for electrical/peripheral behavior that software emulation cannot establish.

This gives a practical ladder from very fast software tests to real hardware without making every change wait for a field test.
