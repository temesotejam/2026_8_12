# CoreS3 / XIAO local simulation experiment

This repository is a deterministic native-C++ simulation and verification environment for M5Stack CoreS3 / XIAO boat software. It is designed to check application behavior, device faults, inter-controller UART behavior, production packet handling, production controller logic, and hardware-log agreement in GitHub Actions without consuming hosted simulator runs.

It is **not** an ESP32-S3 instruction-level or electrical emulator.

## Current layers

```text
Phase 1-6: device/application simulation

scenario
   |
   +--> virtual I2C --> BNO08X / INA226 / VL53L5CX
   +--> virtual UART --> GNSS
   +--> virtual SD / timing faults
   |
   v
portable App logic
   |
   v
SVG + trace.csv

Phase 7: abstract two-controller transport

COMM <==== 921600 bps 8N1 virtual UART ====> CONTROL

Phase 8-9: production protocol/control path

COMM host adapter
   |
   | real boat::Header / packed payload / CRC32 / COBS / 0x00
   v
ProductionUartLink 921600 8N1
   |
   v
CONTROL production boat::Decoder
   |
   +--> production CommandIngress / replay window / Controller::step()
   |
   +--> ControlOutput / ControlSnapshot / INA / VESC / Actuator / Health

Phase 10: hardware-log comparison

real BOAT_*.BIN -----> BOATLOG1 parser -----+
                                             +--> nearest-time comparison
host reference.bin --> BOATLOG1 parser -----+       |
                                                     +--> decoded CSV
                                                     +--> comparison CSV
                                                     +--> summary JSON
                                                     +--> summary SVG
```

## Phases

### Phase 1 — application behavior

BOOT / READY / RUNNING / FAULT, touch handling, IMU fault and recovery.

### Phase 2 — virtual I2C and UART

I2C disconnect/timeout and UART disconnect/delay/frame loss/stale GNSS.

### Phase 3 — transport, timing, and SD faults

I2C NACK, UART corruption/CRC/framing errors, loop jitter, SD write failure/timeout, counters and combined warnings.

### Phase 4 — VirtualBno08x

Startup initialization, report cadence, report stall/stale state, reset, automatic reinitialization, I2C loss during recovery, and last-good attitude behavior.

### Phase 5 — VirtualIna226

Conversion/averaging timing, calibration behavior, V/I/P quantization, stale data, device-specific NACK, reset/reinit, range checks, and math overflow.

### Phase 6 — VirtualVl53l5cx

8x8 / 64-zone distance/status frames, invalid zones, valid-zone count, minimum distance, stale ranging, NACK, reset/reinit, and shared-I2C behavior.

### Phase 7 — two virtual controllers

Full-duplex 921600-bps 8N1 serialization timing, independent direction latency/connectivity, queues, drop/corrupt/framing injection, heartbeats, STOP, and link-loss failsafe.

### Phase 8 — production `boat_protocol`

Copies the packed ABI and codec from the production boat firmware pinned at:

- repository: `temesotejam/2026_8_6`
- source commit: `bf8067c35ee56c884bfbd16e277b27db6e72ef98`

The host suite executes the real production:

- `boat::Type` IDs,
- packed Header/payload ABI,
- CRC32,
- COBS encoder,
- streaming `boat::Decoder`.

The production wire contract is:

```text
Header + payload + CRC32 -> COBS -> 0x00 -> UART 921600 8N1
```

Phase 8 covers real `GnssNavV2`, heartbeat, ARM/DISARM/START/STOP/E-STOP/CLEAR-E-STOP, `CommandPayload`, `CommandAckPayload`, and the bidirectional heartbeat chain.

See `docs/PRODUCTION_PROTOCOL_SOURCE.md`.

### Phase 9 — production command retry and telemetry

Phase 9 additionally compiles the production:

- `production_control`,
- `command_replay`,
- `command_ingress`.

It models:

- `ControlModeCommand`, `ManualCommand`, `HeadingTarget`,
- 64-entry replay window,
- ACK loss -> 100 ms retry using the same request/sequence -> `Duplicate` ACK without reapplication,
- `WaypointSet` / `WaypointAck` duplicate revision handling,
- 1200 ms pending-command timeout,
- separate 200 ms ManualCommand freshness stream with new IDs,
- ARM / START / STOP safety flow,
- 100 ms production telemetry burst in this order:
  `ControlOutput -> ControlSnapshot -> InaStatus -> VescTelemetry -> ActuatorState -> SystemHealth`,
- reverse-link telemetry loss -> COMM heartbeat stops -> CONTROL heartbeat failsafe.

See `docs/PHASE9_PRODUCTION_COMMAND_TELEMETRY.md`.

### Phase 10 — BOATLOG1 hardware-log comparison

Phase 10 parses the compact SD log format currently defined in the open production draft PR #2 (`temesotejam/2026_8_6`, inspected at head `0820d1d1c2107609c839bf6cc5d8c42f9d56e982`). Because that PR is not merged, this is explicitly treated as **draft BOATLOG1 version 1**.

The format has:

- 20-byte little-endian header,
- 40-byte fixed records,
- current draft cadence: 2000 ms,
- logged position, speed, roll/pitch, ToF, left/right/rear command, target/applied duty, waypoint distance, ERPM, safety, mode, yaw criteria, and flags.

`boatlog_replay_cli` compares an observed hardware log to a host-reference log using nearest timestamps and reports:

- time offset,
- position error in metres,
- speed error,
- roll/pitch error,
- ToF error,
- left/right/rear error,
- target/applied duty error,
- waypoint-distance error,
- ERPM error,
- exact safety/mode/yaw/flags mismatches.

Output:

```text
observed_decoded.csv
reference_decoded.csv
comparison.csv
summary.json
summary.svg
```

CI verifies three cases: exact match PASS, 80 ms timestamp shift PASS, and deliberately perturbed data FAIL. See `docs/PHASE10_BOATLOG_REPLAY.md`.

## Build and run

```bash
cmake -S . -B build
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Representative simulator commands:

```bash
./build/cores3_sim scenarios/demo.csv artifacts/demo
./build/dual_controller_sim scenarios/dual_uart_faults.csv artifacts/dual_uart_faults
./build/production_protocol_sim scenarios/production_protocol_faults.csv artifacts/production_protocol_faults
./build/production_command_telemetry_sim scenarios/production_command_retry.csv artifacts/production_command_retry
```

Phase-10 fixture/replay:

```bash
./build/boatlog_fixture artifacts/phase10_fixture

./build/boatlog_replay_cli \
  artifacts/phase10_fixture/observed_exact.bin \
  artifacts/phase10_fixture/reference.bin \
  artifacts/phase10_exact
```

Future real hardware log:

```bash
./build/boatlog_replay_cli \
  BOAT_XXXXXXXX_01.BIN \
  host_reference.bin \
  artifacts/real_run_01
```

## Scenario formats

Single-controller formats remain backward compatible:

- Phase 1: 7 columns
- Phase 2: 12 columns
- Phase 3: 20 columns
- Phase 4: 27 columns
- Phase 5: 39 columns
- Phase 6: 49 columns

Phase 7, Phase 8, and Phase 9 use separate event CSV formats documented by their scenario headers.

## Real CoreS3 build

```bash
pio run -e m5stack-cores3
```

GitHub Actions independently runs the host simulation/test job and the M5Stack CoreS3 PlatformIO build.

## What this can and cannot prove

Strong coverage now includes:

- application state transitions,
- software-visible I2C/UART/device faults,
- deterministic timing policies,
- production packet ABI/CRC/COBS decoding,
- command retry and duplicate suppression,
- production controller calculations compiled natively,
- telemetry scheduling contracts,
- hardware-log observable agreement.

It still does not reproduce:

- ESP32-S3 CPU/register execution,
- FreeRTOS task scheduling and ISR races,
- UART/I2C electrical waveforms,
- pull-ups/capacitance/metastability,
- physical sensor noise/optics,
- RF behavior,
- power/brownout behavior.

The current BOATLOG1 record also does not contain every raw input required to reconstruct the complete production run from first principles. Phase 10 therefore quantifies agreement for **logged observables**; richer deterministic input replay can be added when the hardware log captures command/configuration and complete controller-relevant sensor state.
