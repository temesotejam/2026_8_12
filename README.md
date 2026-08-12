# CoreS3 local simulation experiment

This repository is a proof-of-concept for checking M5Stack CoreS3 application behavior without depending on a hosted simulator for every test run.

The portable C++ application logic is shared by a real CoreS3 Arduino firmware and a desktop simulator. The simulator layers virtual buses and device models behind the same application-facing state.

```text
single-controller device simulation

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

phase-7 two-controller simulation

COMM controller <==== VirtualDuplexUart 921600 8N1 ====> CONTROL controller
      |                                                        |
      +-- GNSS_NAV + heartbeat + STOP                          +-- control result + heartbeat
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

### Phase 7: two virtual controllers over 921600-bps UART

Phase 7 adds a separate deterministic two-controller simulator around `VirtualDuplexUart` and `DualControllerSystem`.

The UART model is full duplex and uses the configured baud rate to calculate serialization time. The default is `921600` baud with `8N1`, so each payload byte consumes ten serial bits. For example, a 92160-byte transfer needs one second of serial time before any extra virtual latency is added.

The two directions are modeled independently:

- communication -> control: `GNSS_NAV`, heartbeat, and `STOP`,
- control -> communication: control result and heartbeat.

Each direction has its own connection state, latency, pending queue, packet/byte counters, injected packet drop, CRC corruption, framing error, and disconnect-drop accounting.

The demo controller policy is:

- GNSS_NAV at 10 Hz,
- control result at 10 Hz,
- heartbeat every 100 ms in both directions,
- heartbeat timeout after 500 ms,
- control-side heartbeat timeout latches a local failsafe stop,
- communication-side heartbeat timeout marks the control link unhealthy,
- a healthy-link `STOP` packet stops the control side independently of the failsafe path,
- `STOP` is queued before regular communication->control traffic generated on the same tick.

A single CRC-corrupted or dropped data frame is visible in counters but does not immediately stop control if heartbeats remain healthy. A long one-way outage eventually removes the corresponding heartbeat and exercises the timeout policy.

Two Phase-7 scenarios are kept separate on purpose:

- `dual_uart_faults.csv` tests packet corruption/drop, one-way disconnect, heartbeat timeout, failsafe stop, reverse-link heartbeat loss, and link recovery,
- `dual_uart_stop.csv` tests STOP propagation on an otherwise healthy link.

Phase-7 SVGs use a 640x300 two-controller view and `trace.csv` records link health, heartbeat age, GNSS/result counts, STOP reception, and per-direction drop/CRC/framing counters.

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

./build/dual_controller_sim scenarios/dual_uart_faults.csv artifacts/dual_uart_faults
./build/dual_controller_sim scenarios/dual_uart_stop.csv artifacts/dual_uart_stop
```

Each simulation writes SVG frames and `trace.csv`.

## Scenario formats

The single-controller device simulator keeps all older formats compatible:

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

The Phase-7 dual-controller simulator uses a separate compact 9-column event format:

```text
t_ms,c2c_connected,c2m_connected,c2c_latency_ms,c2m_latency_ms,c2c_fault,c2m_fault,gnss_valid,send_stop
```

Phase-7 fault codes are `0=none`, `1=drop`, `2=CRC/corrupt`, and `3=framing`.

## Build the real CoreS3 firmware

```bash
pio run -e m5stack-cores3
```

The real firmware uses the same `App` state machine. Virtual device and virtual-link classes are host-simulator components; the CoreS3 firmware continues to use physical hardware drivers.

## GitHub Actions

Every push and pull request runs two independent jobs:

1. **Host simulation**
   - builds the app, bus, device, and dual-controller link models,
   - runs application/integration tests plus Phase 3-7 tests,
   - executes six single-controller scenarios and two Phase-7 dual-controller scenarios,
   - uploads all SVG frames and traces.
2. **CoreS3 firmware build**
   - installs PlatformIO,
   - compiles the real M5Stack CoreS3 Arduino firmware.

## Important limitation

This is not an electrical, optical, RF, or instruction-level emulator. It does not reproduce I2C pull-up/capacitance effects, analog line noise, DMA/interrupt races, exact ESP32-S3 timing, VL53L5CX SPAD/histogram physics, real UART oscillator error or metastability, RF behavior, or power brownouts.

For Phase 7, the UART model intentionally reproduces the software-visible contract: full-duplex serialization time from baud rate and 8N1 framing, queueing, configured latency, packet validation failure, disconnect, heartbeat age, STOP reception, and application-level failsafe behavior. It does not emulate individual UART waveform edges.

## Next useful extensions

The next useful step is to replace the generic Phase-7 packet generators with the actual communication/control packet definitions from the target firmware, then replay real logs through the simulator and compare host-simulated behavior against hardware traces. An interactive front end can sit on top of the same deterministic models afterward.
