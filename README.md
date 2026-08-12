# CoreS3 local simulation experiment

This repository is a proof-of-concept for checking M5Stack CoreS3 application behavior without depending on a hosted simulator for every test run.

## What is simulated

The same portable C++ application logic is shared by:

- a real CoreS3 Arduino firmware (`firmware/src/main.cpp`), and
- a desktop simulator (`simulator/main.cpp`).

The demo application has four states: `BOOT -> READY -> RUNNING`. An IMU failure forces `FAULT`. After the IMU has remained healthy for 500 ms, the app returns to `READY`. A touch in the large bottom button toggles READY/RUNNING.

The desktop simulator feeds scripted sensor/touch inputs from CSV and produces one 320x240 SVG screen image per simulation step, `trace.csv` containing the state transition history, and console output showing the same state transitions.

This is intentionally more useful than a compile-only check: it verifies application state, touch handling, fault behavior, timing logic, and the screen model.

## Run on a PC

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
./build/cores3_sim scenarios/demo.csv artifacts
```

Open the SVG files in `artifacts/` to inspect what the virtual CoreS3 screen showed at each point in the scenario.

## Build the real CoreS3 firmware

```bash
pio run -e m5stack-cores3
```

Upload to hardware when a CoreS3 is connected:

```bash
pio run -e m5stack-cores3 -t upload
```

## Scenario format

`scenarios/demo.csv` columns:

```text
t_ms,pitch_deg,battery_v,imu_ok,touch_pressed,touch_x,touch_y
```

Set `imu_ok` to `0` to inject an IMU failure, or place a touch at `(160,205)` to press the START/STOP button.

## GitHub Actions

Every push and pull request performs two independent checks:

1. **Host simulation**: builds the shared logic, runs deterministic tests, executes the scripted scenario, and uploads the SVG frames + trace as an artifact.
2. **CoreS3 firmware build**: installs PlatformIO and compiles the real Arduino firmware for M5Stack CoreS3.

The `cores3-sim-frames` artifact is the first thing to inspect when you want to verify behavior rather than merely compilation.

## Important limitation

This does **not** emulate the ESP32-S3 CPU, I2C electrical behavior, LCD controller timings, Wi-Fi radio, or all M5Stack peripherals. Instead it moves the important application logic behind a hardware-independent boundary so that behavior can be tested deterministically on a PC. Real-hardware tests are still required for driver, timing, power, RF, and electrical problems.

## Next step

Once this proof-of-concept is stable, the same pattern can be extended with virtual UART, I2C devices, GNSS streams, sensor dropouts, latency/jitter injection, SD-card failures, and richer interactive visualization.
