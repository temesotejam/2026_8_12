# CoreS3 local simulation experiment

This repository is a proof-of-concept for checking M5Stack CoreS3 application behavior without depending on a hosted simulator for every test run.

## What is simulated

The same portable C++ application logic is shared by:

- a real CoreS3 Arduino firmware (`firmware/src/main.cpp`), and
- a desktop simulator (`simulator/main.cpp`).

The demo application has four states: `BOOT -> READY -> RUNNING`. A critical I2C/IMU failure forces `FAULT`. After the critical sensor path has remained healthy for 500 ms, the app returns to `READY`. A touch in the large bottom button toggles READY/RUNNING.

Phase 2 adds a virtual hardware layer instead of injecting sensor values directly into the app:

```text
scenario
   |
   +--> virtual I2C bus --> virtual IMU --> SensorSample
   |
   +--> virtual UART ----> GNSS stream --> SensorSample
                                     |
                                     v
                                  App logic
                                     |
                                     v
                              SVG screen + trace
```

The desktop simulator now verifies application state, touch handling, bus timeouts, device disconnects, UART delay/drop behavior, stale GNSS data, recovery timing, and the screen model.

## Virtual I2C behavior

The virtual IMU is read through a simulated I2C path.

- `i2c_connected=0` simulates a disconnected/stuck bus.
- `i2c_latency_ms > 10` simulates a transaction exceeding the configured 10 ms timeout.
- `imu_online=0` simulates a device that is not responding even though the bus itself is usable.
- A failed read holds the last good pitch value instead of inventing a new measurement.
- I2C/IMU failure is treated as critical and forces the application into `FAULT`.

The simulator also counts I2C reads and timeouts.

## Virtual UART / GNSS behavior

Each scenario step can inject a GNSS frame through a virtual UART.

- `uart_connected=0` drops new frames and clears in-flight frames.
- `uart_latency_ms` delays when a GNSS frame becomes visible to the application.
- `gnss_valid=0` means the GNSS source emits no new frame on that step.
- GNSS data becomes stale 500 ms after the most recently delivered frame.

For this proof of concept, UART/GNSS loss is **warning-only** and does not stop local control. That policy is deliberate so the simulator can distinguish a critical control sensor from a non-critical communication path. It can be changed later for the actual system requirements.

## Run on a PC

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure

./build/cores3_sim scenarios/demo.csv artifacts/demo
./build/cores3_sim scenarios/bus_faults.csv artifacts/bus_faults
```

Open the SVG files in `artifacts/` to inspect what the virtual CoreS3 screen showed at each point. Each scenario also writes a `trace.csv` with state, bus health, GNSS age, warning text, and error counters.

## Build the real CoreS3 firmware

```bash
pio run -e m5stack-cores3
```

Upload to hardware when a CoreS3 is connected:

```bash
pio run -e m5stack-cores3 -t upload
```

The real firmware uses the same `App` state machine. The virtual-bus implementation is for the PC simulator; the real firmware continues to read the CoreS3 IMU through M5Unified.

## Scenario formats

The original 7-column format remains supported:

```text
t_ms,pitch_deg,battery_v,imu_ok,touch_pressed,touch_x,touch_y
```

The new bus-aware 12-column format is:

```text
t_ms,pitch_deg,battery_v,imu_online,i2c_connected,i2c_latency_ms,uart_connected,uart_latency_ms,gnss_valid,touch_pressed,touch_x,touch_y
```

See `scenarios/bus_faults.csv` for a complete fault-injection example.

## GitHub Actions

Every push and pull request performs two independent checks:

1. **Host simulation**
   - builds the shared application and virtual hardware layers,
   - runs application, virtual-bus, and integration tests,
   - runs both the normal and bus-fault scenarios,
   - uploads all SVG frames and trace CSV files.
2. **CoreS3 firmware build**
   - installs PlatformIO,
   - compiles the real Arduino firmware for M5Stack CoreS3.

The `cores3-sim-artifacts` artifact is the first thing to inspect when you want to verify behavior rather than merely compilation.

## What this proves

This approach can detect many logic-level faults without a physical CoreS3:

- wrong state transition after a sensor failure,
- missing safety lockout,
- bad recovery timing,
- application behavior during I2C timeouts,
- application behavior during UART disconnects,
- delayed/stale data handling,
- UI/state mismatch,
- regressions introduced by later code changes.

## Important limitation

This still does **not** electrically emulate the ESP32-S3 or the real I2C/UART peripheral hardware. It does not reproduce pull-up resistance, bus capacitance, line noise, clock stretching bugs, DMA/interrupt races, RF behavior, power brownouts, or the exact LCD controller timing.

Instead it emulates the *observable contract* of those peripherals: success, timeout, disconnect, delay, stale data, and recovery. That is enough to test a large fraction of application behavior deterministically and without hosted-simulator run limits.

## Next useful extensions

The same virtual-hardware layer can now be extended with:

- UART byte corruption / framing errors,
- I2C NACK at a selected transaction number,
- timing jitter,
- SD-card write failures / full-card behavior,
- virtual ToF / INA226 / BNO08X devices,
- scripted serial traffic between two virtual controllers,
- an interactive browser front end for changing inputs while the simulation runs.
