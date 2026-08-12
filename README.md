# CoreS3 local simulation experiment

This repository is a proof-of-concept for checking M5Stack CoreS3 application behavior without depending on a hosted simulator for every test run.

The portable C++ application logic is shared by a real CoreS3 Arduino firmware and a desktop simulator. The simulator layers virtual buses and device models behind the same application-facing state.

```text
scenario
   |
   +--> virtual I2C bus --> VirtualBno08x ----> SensorSample
   |                   |-> VirtualIna226 ----> SensorSample
   |                   \-> VirtualVl53l5cx --> SensorSample
   |
   +--> virtual UART ----> GNSS stream ------> SensorSample
   +--> virtual SD + timing health ----------> SensorSample
                                               |
                                               v
                                            App logic
                                               |
                                               v
                                        SVG screen + trace
```

## Phases

### Phase 1: application behavior

The original 7-column scenario checks state transitions, touch handling, and direct IMU fault behavior.

### Phase 2: virtual I2C and UART

Adds I2C disconnect/timeout plus UART disconnect, delay, frame loss, and stale GNSS data.

### Phase 3: transport, timing, and SD faults

Adds I2C NACK, UART byte/checksum/framing corruption, control-loop jitter, SD removal/write failure/write timeout, and diagnostic counters.

### Phase 4: VirtualBno08x

Models startup initialization, configurable report interval, report-stream stall, reset, automatic host reinitialization, I2C loss during reinit, report age/stale detection, last-good attitude retention, and device counters. BNO08X is on the critical control path, so runtime BNO/I2C loss can force `FAULT`.

### Phase 5: VirtualIna226

Models INA226 conversion timing and averaging, completed-conversion register updates, Current_LSB/Power_LSB quantization, calibration-dependent current/power output, stale conversions, device-specific NACK, reset/reinit, range checks, and math-overflow diagnostics. INA226 is monitoring-only in the current policy, so INA-only faults produce warnings without stopping local control.

### Phase 6: VirtualVl53l5cx

Adds a device-specific 8x8 VL53L5CX model on the same virtual I2C transport.

The model covers:

- 64-zone frame output,
- 8x8 ranging-frequency control capped at 15 Hz in this simulator,
- one distance and target-status value per zone,
- per-zone validity and valid-zone count,
- minimum valid distance across the frame,
- frame age and stale-frame detection,
- ranging stop and frame-stream stall,
- device-specific ACK/NACK independent of BNO08X/INA226,
- power loss, reset, automatic host reinitialization, and retry after I2C recovery,
- counters for reset, reinit, delivered frames, stale events, device NACK, and injected invalid-zone events.

The proof-of-concept treats target statuses `5`, `6`, `9`, and `12` as usable application results and rejects distances outside the modeled 2..4000 mm range. This is an application-policy model, not an optical simulator.

VL53L5CX-only problems are warning-only in the current policy so they can be distinguished from a shared I2C failure. A shared I2C loss still becomes critical because the same bus also feeds the control IMU path.

SVG output distinguishes `TOF:INIT`, `TOF:RUN`, `TOF:PART`, `TOF:STALE`, `TOF:OFF`, and `TOF:N/A`, and shows valid zones plus minimum valid distance.

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
./build/cores3_sim scenarios/vl53l5cx_faults.csv artifacts/vl53l5cx_faults
```

Each simulation writes 320x240 SVG frames and `trace.csv`.

## Scenario formats

Older formats stay compatible:

- Phase 1: 7 columns
- Phase 2: 12 columns
- Phase 3: 20 columns
- Phase 4: 27 columns
- Phase 5: 39 columns
- Phase 6: 49 columns

Phase 6 appends these ten columns to the exact Phase-5 format:

```text
tof_model,tof_powered,tof_device_ack,tof_reset,tof_ranging_enabled,tof_stall_frames,tof_frequency_hz,tof_default_distance_mm,tof_invalid_zone,tof_invalid_status
```

See `scenarios/vl53l5cx_faults.csv` for partial-zone failure, frame stall/stale detection, device-only NACK/reinit, reset, shared-I2C loss during recovery, and recovery examples.

## Build the real CoreS3 firmware

```bash
pio run -e m5stack-cores3
```

The real firmware uses the same `App` state machine. Virtual device classes are host-simulator components; the CoreS3 firmware continues to use physical hardware drivers.

## GitHub Actions

Every push and pull request runs two independent jobs:

1. **Host simulation**
   - builds shared app + bus + device models,
   - runs application, integration, and Phase 3-6 tests,
   - executes all six scenario generations,
   - uploads all SVG frames and traces.
2. **CoreS3 firmware build**
   - installs PlatformIO,
   - compiles the real M5Stack CoreS3 Arduino firmware.

## Important limitation

This is not an electrical, optical, or instruction-level emulator. It does not reproduce I2C pull-up/capacitance effects, analog line noise, DMA/interrupt races, exact ESP32-S3 timing, VL53L5CX SPAD/histogram physics, cover-glass crosstalk, ambient-light effects, physical shunt error, RF behavior, or power brownouts.

It intentionally models the software-visible contract: initialization, completed data updates, validity, stale data, disconnect/NACK, reset/reconfiguration, warnings, criticality policy, and recovery.

## Next useful extensions

The next useful step is scripted UART traffic between two virtual controllers, followed by an interactive front end for changing sensor/bus states while the deterministic simulator runs.
