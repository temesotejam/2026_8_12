# CoreS3 local simulation experiment

This repository is a proof-of-concept for checking M5Stack CoreS3 application behavior without depending on a hosted simulator for every test run.

## What is simulated

The same portable C++ application logic is shared by:

- a real CoreS3 Arduino firmware (`firmware/src/main.cpp`), and
- a desktop simulator (`simulator/main.cpp`).

The demo application has four states: `BOOT -> READY -> RUNNING`. Critical control-sensor failure forces `FAULT`. After the critical path has remained healthy for 500 ms, the app returns to `READY`.

The virtual hardware path is now layered by transport and device:

```text
scenario
   |
   +--> virtual I2C bus --> virtual BNO08X --> SensorSample
   |
   +--> virtual UART ----> GNSS stream ----> SensorSample
   |
   +--> virtual SD + timing health --------> SensorSample
                                           |
                                           v
                                        App logic
                                           |
                                           v
                                    SVG screen + trace
```

## Phase 1: application behavior

The original 7-column scenario directly describes sensor/touch behavior. It remains supported so older tests continue to run unchanged.

## Phase 2: virtual I2C and UART

The virtual I2C path supports disconnect and transaction timeout. The virtual UART/GNSS path supports disconnect, delivery latency, dropped frames, and stale-data detection.

## Phase 3: NACK, corrupt frames, jitter, and SD failures

Phase 3 adds I2C NACK, UART byte/checksum/framing corruption, signed control-loop jitter, virtual SD removal/write failure/write timeout, dedicated counters, and combined warnings.

## Phase 4: device-specific BNO08X model

Phase 4 replaces the generic IMU with an optional dedicated `VirtualBno08x` model. Earlier scenarios keep the direct IMU path unless `bno_model=1` is supplied.

The BNO08X model represents the host-visible behavior needed by the control application:

- configurable report interval (`bno_report_interval_ms`),
- startup initialization and host reconfiguration delay (`bno_reinit_delay_ms`),
- report enable/disable,
- report-stream stall without an I2C disconnect,
- reset as an edge-triggered event,
- automatic reinitialization after reset,
- cancellation/retry of reinitialization if I2C fails during recovery,
- last good pitch retention,
- report age and stale detection,
- counters for resets, initialization attempts/successes, delivered reports, and stale events.

The current default BNO08X scenario uses a 20 ms report interval. A report is considered stale after the larger of 100 ms or five configured report periods. These are simulator policy parameters rather than claims about the electrical device.

### Startup behavior

During normal boot, BNO08X initialization is treated as part of `BOOT`, not as an immediate fault. The application waits for a fresh report before entering `READY`.

After the system has started, however, a BNO08X reset, missing/stale report stream, I2C failure, or missing device is critical and forces `FAULT`. Once the BNO08X is healthy again, the existing 500 ms application recovery gate still applies.

### BNO08X states shown in SVG output

- `BNO:INIT` — host reinitialization in progress,
- `BNO:RUN` — initialized with a fresh report,
- `BNO:STALE` — initialized but report data is too old,
- `BNO:OFF` — no initialized device/report path,
- `BNO:N/A` — old Phase 1-3 direct-IMU mode.

## Run on a PC

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure

./build/cores3_sim scenarios/demo.csv artifacts/demo
./build/cores3_sim scenarios/bus_faults.csv artifacts/bus_faults
./build/cores3_sim scenarios/transport_faults.csv artifacts/transport_faults
./build/cores3_sim scenarios/bno08x_faults.csv artifacts/bno08x_faults
```

Every simulation writes 320x240 SVG screen frames and a `trace.csv`. The Phase-4 trace adds BNO08X initialization/report health and device-model counters.

## Build the real CoreS3 firmware

```bash
pio run -e m5stack-cores3
```

Upload to hardware when a CoreS3 is connected:

```bash
pio run -e m5stack-cores3 -t upload
```

The real firmware uses the same `App` state machine. Virtual device classes are compiled only into the host simulator; the real CoreS3 firmware continues to use M5Unified for physical hardware.

## Scenario formats

Phase 1, 7 columns:

```text
t_ms,pitch_deg,battery_v,imu_ok,touch_pressed,touch_x,touch_y
```

Phase 2, 12 columns:

```text
t_ms,pitch_deg,battery_v,imu_online,i2c_connected,i2c_latency_ms,uart_connected,uart_latency_ms,gnss_valid,touch_pressed,touch_x,touch_y
```

Phase 3, 20 columns:

```text
t_ms,pitch_deg,battery_v,imu_online,i2c_connected,i2c_latency_ms,i2c_nack,uart_connected,uart_latency_ms,gnss_valid,uart_corrupt_byte,uart_crc_error,uart_framing_error,loop_jitter_ms,sd_connected,sd_fail_write,sd_latency_ms,touch_pressed,touch_x,touch_y
```

Phase 4, 27 columns, appends:

```text
bno_model,bno_reset,bno_reports_enabled,bno_stall_reports,bno_report_interval_ms,bno_reinit_delay_ms,bno_auto_reinit
```

See `scenarios/bno08x_faults.csv` for startup initialization, report stall, reset/recovery, and I2C-loss-during-reinitialization examples.

## GitHub Actions

Every push and pull request performs two independent checks:

1. **Host simulation**
   - builds shared app + transport + device models,
   - runs application, virtual-hardware, integration, Phase-3, and Phase-4 tests,
   - executes all four scenario generations,
   - uploads all SVG frames and trace CSV files.
2. **CoreS3 firmware build**
   - installs PlatformIO,
   - compiles the real Arduino firmware for M5Stack CoreS3.

The `cores3-sim-artifacts` artifact is the first thing to inspect when verifying behavior rather than merely compilation.

## Important limitation

This is not an electrical or instruction-level emulator. It does not reproduce pull-up resistance, bus capacitance, analog line noise, clock stretching, DMA/interrupt races, exact ESP32-S3 timing, RF behavior, brownouts, or LCD controller timing.

The BNO08X model also does not attempt to reproduce every SH-2 feature. It currently models the fused attitude-report path that matters to this control demonstration: configuration/reinitialization, report timing, stale data, reset, transport availability, and recovery.

## Next useful extensions

The same device-model pattern can now be used for:

- INA226 conversion timing, current/power values, overflow and I2C loss,
- VL53L5CX frame timing, zone data, stale frames and restart behavior,
- multiple BNO08X report streams (rotation vector, accelerometer, gyroscope),
- scripted UART traffic between two virtual controllers,
- an interactive browser front end on top of the deterministic simulator.
