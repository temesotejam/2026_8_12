# CoreS3 local simulation experiment

This repository is a proof-of-concept for checking M5Stack CoreS3 application behavior without depending on a hosted simulator for every test run.

## What is simulated

The same portable C++ application logic is shared by:

- a real CoreS3 Arduino firmware (`firmware/src/main.cpp`), and
- a desktop simulator (`simulator/main.cpp`).

The demo application has four states: `BOOT -> READY -> RUNNING`. Critical control-sensor failure forces `FAULT`. After the critical path has remained healthy for 500 ms, the app returns to `READY`.

The virtual hardware path is layered by transport and device:

```text
scenario
   |
   +--> virtual I2C bus --> virtual BNO08X --> SensorSample
   |                   \-> virtual INA226 --> SensorSample
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

During normal boot, BNO08X initialization is treated as part of `BOOT`, not as an immediate fault. After startup, a BNO08X reset, missing/stale report stream, I2C failure, or missing device is critical and forces `FAULT`.

BNO08X states shown in SVG output are `INIT`, `RUN`, `STALE`, `OFF`, and `N/A`.

## Phase 5: device-specific INA226 model

Phase 5 adds a dedicated `VirtualIna226` on the same virtual I2C transport. The INA226 is treated as a monitoring device in the current demo policy, so its faults produce warnings and diagnostics without stopping local control. A shared I2C-bus failure can still be critical through the BNO08X/control-sensor path.

The model follows the host-visible conversion behavior of the INA226 rather than trying to emulate its analog front end electrically:

- separate shunt and bus conversion-time settings,
- supported conversion times of 140, 204, 332, 588 us and 1.1, 2.116, 4.156, 8.244 ms,
- averaging selections of 1, 4, 16, 64, 128, 256, 512, and 1024 samples,
- output values update only when the configured shunt + bus conversion and averaging cycle completes,
- register values remain at their previous completed conversion while a new averaging cycle is still in progress,
- current and power are zero when calibration is not programmed,
- fixed bus-voltage and shunt-voltage quantization in the simulator (1.25 mV and 2.5 uV),
- current quantization from the configured Current_LSB and power quantization at 25 times Current_LSB,
- bus-voltage range checking from 0 to 36 V,
- shunt-voltage range checking around +/-81.92 mV,
- device-specific ACK/NACK independent of other devices on the same virtual I2C bus,
- power loss, reset, host reinitialization, and automatic retry after the device returns,
- conversion-stream stall and stale-data detection,
- calibration-missing, range-error, and injected math-overflow diagnostics,
- counters for resets, initialization attempts/successes, completed conversions, stale events, device NACKs, configuration/range errors, math overflow, and uncalibrated conversions.

The Phase-5 example uses 588 us shunt conversion + 588 us bus conversion with 16-sample averaging, which gives an approximately 18.8 ms complete conversion cycle.

INA226 states shown in SVG output are:

- `INA:INIT` — host configuration/reinitialization in progress,
- `INA:RUN` — initialized with a fresh completed conversion,
- `INA:STALE` — initialized but no recent completed conversion,
- `INA:OFF` — device unavailable/uninitialized,
- `INA:CAL` — calibration is not programmed,
- `INA:RANGE` — simulated input is outside the modeled input range,
- `INA:OVF` — math-overflow diagnostic is asserted,
- `INA:N/A` — INA226 model is disabled for an older scenario.

## Run on a PC

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure

./build/cores3_sim scenarios/demo.csv artifacts/demo
./build/cores3_sim scenarios/bus_faults.csv artifacts/bus_faults
./build/cores3_sim scenarios/transport_faults.csv artifacts/transport_faults
./build/cores3_sim scenarios/bno08x_faults.csv artifacts/bno08x_faults
./build/cores3_sim scenarios/ina226_faults.csv artifacts/ina226_faults
```

Every simulation writes 320x240 SVG screen frames and a `trace.csv`. Phase 5 adds INA226 initialization/conversion health, V/I/P/shunt values, measurement age, and INA226 device counters to the trace.

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

Phase 5, 39 columns, appends:

```text
ina_model,ina_powered,ina_device_ack,ina_reset,ina_stall,ina_calibration,ina_math_overflow,ina_bus_voltage_v,ina_current_a,ina_conversion_us,ina_averages,ina_reinit_delay_ms
```

See `scenarios/ina226_faults.csv` for conversion timing, calibration loss/recovery, stale conversion data, INA-only NACK/reinit, range error, math overflow, reset, and recovery examples.

## GitHub Actions

Every push and pull request performs two independent checks:

1. **Host simulation**
   - builds shared app + transport + device models,
   - runs application, virtual-hardware, integration, Phase-3, Phase-4, and Phase-5 tests,
   - executes all five scenario generations,
   - uploads all SVG frames and trace CSV files.
2. **CoreS3 firmware build**
   - installs PlatformIO,
   - compiles the real Arduino firmware for M5Stack CoreS3.

The `cores3-sim-artifacts` artifact is the first thing to inspect when verifying behavior rather than merely compilation.

## Important limitation

This is not an electrical or instruction-level emulator. It does not reproduce pull-up resistance, bus capacitance, analog line noise, clock stretching, DMA/interrupt races, exact ESP32-S3 timing, RF behavior, brownouts, or LCD controller timing.

The BNO08X model does not reproduce every SH-2 feature, and the INA226 model does not reproduce analog error, real ADC noise, thermal drift, physical shunt tolerance, or every alert-pin behavior. The models focus on the observable software contract needed for deterministic application testing.

## Next useful extensions

The next device-specific step is a virtual VL53L5CX model with frame timing, multi-zone data, stale-frame handling, restart behavior, invalid/range-status zones, and I2C recovery. After that, scripted UART traffic between two virtual controllers can exercise the full two-controller architecture.
