# Test scope

This repository deliberately separates different confidence levels instead of calling every green check a full hardware simulation.

## 1. CoreS3 hardware-topology contract

A passing topology check proves that the machine-readable board map matches the captured CoreS3 wiring contract used by the simulator.

It validates:

- Port A on the CoreS3 main unit: GPIO2 SDA / GPIO1 SCL,
- Port B through Base DIN/M5-Bus: GPIO9 PB_OUT / GPIO8 PB_IN,
- Port C through Base DIN/M5-Bus: GPIO17 PC_TX / GPIO18 PC_RX,
- all 30 M5-Bus positions,
- LCD + microSD shared SPI wiring,
- internal I2C GPIO12/GPIO11 and onboard device addresses,
- BMM150 behind the BMI270 auxiliary bus,
- touch reset/interrupt routing through AW9523B,
- camera control/data/timing pins,
- audio I2S wiring,
- RTC/IMU identities and addresses.

The source is `sim/cores3/board_topology.json`; `tools/validate_cores3_topology.py` checks the invariants in CI.

This is a **connection model**, not yet behavioral emulation of each chip.

## 2. Native shared-logic test

A passing native test gives strong evidence that deterministic, hardware-independent C++ logic behaves as specified. It is ideal for state machines, parsers, timeout logic, navigation math, filtering, validation rules, and large scenario sets.

It does **not** exercise the ESP32-S3 CPU, FreeRTOS, flash boot, Arduino, M5Unified, or hardware registers.

## 3. ESP32-S3 QEMU runtime test

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

It does **not** yet mean M5Unified's CoreS3-specific LCD/touch/audio/camera path is behaviorally modeled. The board topology is now known, so the next work is attaching virtual peripheral models to those real connection points.

## 4. Real CoreS3 PlatformIO build

A passing CoreS3 build proves that the real Arduino/M5Unified application remains compatible with the shared code and produces firmware for the `m5stack-cores3` target.

Without a physical board or completed CoreS3 peripheral models, build success alone does not prove the external hardware path at runtime.

## Optional Wokwi test

Wokwi remains an optional comparison check. When `WOKWI_CLI_TOKEN` is configured, it can run the CoreS3-oriented firmware in Wokwi's CoreS3 model. The self-hosted native, topology, and QEMU jobs do not require this token.

## Peripheral-emulation status

The topology is already captured; the following behavioral models are still pending:

- ILI9342C command parser and 320x240 framebuffer,
- AW9523B register behavior needed by M5Unified startup/reset paths,
- AXP2101 subset needed by LCD power/backlight and board startup,
- FT6336U touch coordinate injection,
- microSD SPI storage transactions,
- attachable Port A/B/C endpoints,
- BMI270/BMM150 data generation,
- camera pixel stream,
- audio codec/amplifier behavior.

The intended LCD end-to-end path is:

```text
real CoreS3/M5Unified drawing calls
        -> ESP32-S3 SPI/GPIO traffic
        -> virtual ILI9342C
        -> 320x240 framebuffer
        -> cores3-screen.png artifact
```

## What no virtual test will fully prove

Even after the digital device models are complete, software simulation will not establish:

- real power sequencing, voltage droop or brownout margins,
- electrical I2C/SPI/UART signal integrity,
- pull-up resistor sizing and cable capacitance,
- EMI/noise coupling,
- actual CoreS3 thermal limits,
- GNSS RF reception or antenna performance,
- physical sensor bias, vibration, temperature drift or magnetic interference.

Those remain real-hardware/HIL tests.
