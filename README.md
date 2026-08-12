# CoreS3 local simulation experiment

This repository is a proof-of-concept for checking M5Stack CoreS3 application behavior without depending on a hosted simulator for every test run.

## What is simulated

The same portable C++ application logic is shared by:

- a real CoreS3 Arduino firmware (`firmware/src/main.cpp`), and
- a desktop simulator (`simulator/main.cpp`).

The demo application has four states: `BOOT -> READY -> RUNNING`. A critical I2C/IMU failure forces `FAULT`. After the critical sensor path has remained healthy for 500 ms, the app returns to `READY`. A touch in the large bottom button toggles READY/RUNNING.

The virtual hardware path is:

```text
scenario
   |
   +--> virtual I2C bus --> virtual IMU --> SensorSample
   |
   +--> virtual UART ----> GNSS stream --> SensorSample
   |
   +--> virtual SD + timing health ------> SensorSample
                                     |
                                     v
                                  App logic
                                     |
                                     v
                              SVG screen + trace
```

The simulator verifies state transitions, touch handling, bus failures, communication corruption, stale data, timing jitter, logging failures, recovery behavior, and the screen model.

## Phase 1: application behavior

The original 7-column scenario directly describes sensor/touch behavior. It remains supported so older tests continue to run unchanged.

## Phase 2: virtual I2C and UART

The virtual IMU is read through a simulated I2C path.

- `i2c_connected=0` simulates a disconnected/stuck bus.
- `i2c_latency_ms > 10` simulates a transaction exceeding the 10 ms timeout.
- `imu_online=0` simulates a missing device on an otherwise usable bus.
- Failed reads retain the last good pitch value.
- I2C/IMU failure is critical and forces `FAULT`.

The virtual UART/GNSS path supports disconnect, delivery latency, dropped frames, and 500 ms stale-data detection. UART/GNSS loss is warning-only in this proof of concept so failure criticality can be tested independently from the control IMU.

## Phase 3: NACK, corrupt frames, jitter, and SD failures

Phase 3 extends the observable hardware contract without trying to electrically emulate the ESP32-S3.

### I2C NACK

`i2c_nack=1` simulates a device address/transaction that is not acknowledged. It is distinguished from a timeout/disconnect in the UI and trace, increments a dedicated NACK counter, retains the last good pitch value, and forces `FAULT` because the IMU sample is unavailable.

### UART data corruption

A GNSS frame can be rejected for three separate causes:

- `uart_corrupt_byte=1`: one byte is corrupted and the checksum rejects the frame,
- `uart_crc_error=1`: checksum/CRC validation fails,
- `uart_framing_error=1`: the UART frame itself is malformed.

Rejected frames do not refresh GNSS age. The link can remain physically connected while the application reports `UART frame error`. These faults are warning-only in the current demo policy.

### Control-loop jitter

`loop_jitter_ms` injects signed timing error. Absolute jitter above 20 ms sets `TIME:BAD`, raises a `Control loop jitter` warning, increments a jitter-event counter, and tracks maximum observed jitter. It does not force `FAULT` in the current policy.

### SD write failures

The simulator performs one virtual log write per simulation step.

- `sd_connected=0` simulates card removal,
- `sd_fail_write=1` simulates an explicit write failure,
- `sd_latency_ms > 20` simulates a write timeout.

These conditions set `SD:BAD`, increment failure/timeout counters, and raise `SD write failed` without stopping local control.

## Run on a PC

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure

./build/cores3_sim scenarios/demo.csv artifacts/demo
./build/cores3_sim scenarios/bus_faults.csv artifacts/bus_faults
./build/cores3_sim scenarios/transport_faults.csv artifacts/transport_faults
```

Every simulation writes 320x240 SVG screen frames and a `trace.csv`. The trace includes state, bus health, GNSS age, UART-frame health, timing status, SD status, warning text, and cumulative error counters.

## Build the real CoreS3 firmware

```bash
pio run -e m5stack-cores3
```

Upload to hardware when a CoreS3 is connected:

```bash
pio run -e m5stack-cores3 -t upload
```

The real firmware uses the same `App` state machine. The virtual hardware implementation exists only in the PC simulator; real CoreS3 code continues to use M5Unified for the physical IMU and display.

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

See `scenarios/transport_faults.csv` for the combined Phase-3 example.

## GitHub Actions

Every push and pull request performs two independent checks:

1. **Host simulation**
   - builds the shared app and virtual hardware layers,
   - runs application, virtual-bus, integration, and Phase-3 fault tests,
   - runs all three scenario formats,
   - uploads all SVG frames and trace CSV files.
2. **CoreS3 firmware build**
   - installs PlatformIO,
   - compiles the real Arduino firmware for M5Stack CoreS3.

The `cores3-sim-artifacts` artifact is the first thing to inspect when verifying behavior rather than merely compilation.

## What this can catch

This environment can detect logic-level problems such as wrong safety state transitions, missing lockout, incorrect recovery timing, stale-data handling mistakes, UART error handling mistakes, control-loop jitter policy regressions, SD logging failure handling, UI/state mismatch, and later software regressions.

## Important limitation

This is not an electrical emulator. It does not reproduce pull-up resistance, bus capacitance, analog line noise, clock stretching, DMA/interrupt races, exact ESP32-S3 timing, RF behavior, brownouts, or LCD controller timing. Real hardware is still required for those checks.

The simulator instead emulates the observable contract of the hardware: success, timeout, NACK, corruption, framing failure, delay, stale data, timing error, storage failure, and recovery.

## Next useful extensions

The next useful step is to build device-specific models on top of this transport layer, for example virtual BNO08X, VL53L5CX, INA226, and scripted serial traffic between two virtual controllers. An interactive browser front end can then sit on top of the same deterministic simulator.
