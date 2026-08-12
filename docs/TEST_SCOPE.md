# Test scope

This repository is deliberately a first-stage smoke test, not a claim that Wokwi replaces a physical CoreS3.

## What a passing Wokwi test tells us

A pass gives evidence that:

- the PlatformIO project can produce a CoreS3 firmware image,
- the ESP32-S3 firmware boots in the Wokwi CoreS3 model,
- M5Unified initialization reaches the application code,
- the display API is usable enough for the firmware to obtain a non-zero display size and draw text,
- `setup()` completes,
- `loop()` keeps executing over several seconds,
- `millis()`/timing logic used by the heartbeat behaves as expected,
- USB Serial output reaches the simulator,
- GitHub Actions can automatically judge an expected runtime result instead of checking compilation only.

## What this does not prove

A pass does not prove:

- real power sequencing or brownout behavior,
- electrical I2C/SPI/UART signal integrity,
- pull-up resistor correctness,
- EMI/noise behavior,
- actual CoreS3 thermal or CPU-load limits,
- physical display/touch/camera/speaker behavior beyond what Wokwi models,
- GNSS RF reception or antenna performance,
- physical sensor bias, vibration, temperature drift, or magnetic interference.

Those still need real-hardware tests.

## Planned extension path

Once this smoke test is reliable, the same structure can be extended without replacing the application firmware:

1. add a virtual UART GNSS device,
2. feed deterministic NMEA/UBX data,
3. add GNSS dropout/corruption/recovery cases,
4. add virtual I2C sensor behavior,
5. record machine-readable firmware state over Serial,
6. fail CI when state transitions, rates, timeouts, or outputs differ from the expected behavior.

That is the intended path from this simple proof to a useful pre-hardware regression test environment.
