# CoreS3 local simulation experiment

This repository is a deterministic native-C++ simulation environment for checking M5Stack CoreS3 / XIAO application behavior without depending on a hosted simulator for every run. It is a behavior- and protocol-level simulator, not an ESP32-S3 instruction or electrical emulator.

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

phase-7 abstract two-controller simulation

COMM controller <==== VirtualDuplexUart 921600 8N1 ====> CONTROL controller

phase-8 production-protocol simulation

COMM XIAO
   |  boat::Header + real payload + CRC32
   |  COBS encoded + 0x00 delimiter
   |  921600 baud / 8N1
   v
ProductionUartLink
   |
   v
CONTROL XIAO production boat::Decoder
```

## Phases

### Phase 1: application behavior

Checks BOOT/READY/RUNNING/FAULT transitions, touch handling, and direct IMU fault behavior.

### Phase 2: virtual I2C and UART

Adds I2C disconnect/timeout plus UART disconnect, delay, frame loss, and stale GNSS data.

### Phase 3: transport, timing, and SD faults

Adds I2C NACK, UART byte/checksum/framing corruption, control-loop jitter, SD removal/write failure/write timeout, and diagnostic counters.

### Phase 4: VirtualBno08x

Models startup initialization, report interval, report stall, reset, host reinitialization, I2C loss during reinit, report age/stale detection, last-good attitude retention, and recovery. Runtime BNO/I2C loss is critical and can force `FAULT`.

### Phase 5: VirtualIna226

Models INA226 conversion timing/averaging, completed-conversion updates, Current_LSB/Power_LSB quantization, calibration-dependent current/power, stale conversions, device-specific NACK, reset/reinit, range checks, and math-overflow diagnostics. INA-only faults are warning-only in the current demo policy.

### Phase 6: VirtualVl53l5cx

Models an 8x8 / 64-zone VL53L5CX software contract: frame frequency, distance/status per zone, invalid zones, valid-zone count, minimum distance, stale frames, ranging stop/stall, device NACK, reset/reinit, and I2C recovery. VL53L5CX-only failures are warning-only; shared I2C loss remains critical through the control IMU path.

### Phase 7: two virtual controllers over 921600-bps UART

Adds `VirtualDuplexUart` and `DualControllerSystem` as an abstract two-controller link model.

- full duplex, 921600 baud, 8N1 serialization time
- independent direction latency/connectivity
- queue accounting
- drop / corrupt / framing injection
- 10 Hz abstract GNSS/result traffic
- 100 ms heartbeats
- 500 ms heartbeat timeout
- healthy-link STOP and link-loss failsafe tests

Phase 7 deliberately uses abstract packet types. It remains useful for transport stress testing and bandwidth tests.

### Phase 8: production `boat_protocol` on the virtual UART

Phase 8 replaces the abstract Phase-7 message format with the real boat UART protocol copied from the production two-XIAO firmware.

Pinned production source:

- repository: `temesotejam/2026_8_6`
- source commit: `bf8067c35ee56c884bfbd16e277b27db6e72ef98`
- production header: `control/lib/boat_protocol/src/boat_protocol.h`
- production codec: `control/lib/boat_protocol/src/boat_protocol.cpp`

See `docs/PRODUCTION_PROTOCOL_SOURCE.md` for source blob hashes and exact provenance.

Phase 8 compiles the copied production codec as `boat_protocol_host`. The simulator therefore uses the same:

- packed protocol ABI,
- `boat::Type` numeric IDs,
- CRC32 implementation,
- COBS encoder,
- streaming `boat::Decoder`,
- maximum payload and frame limits.

The production wire frame is:

```text
packed Header (22 B)
+ payload
+ CRC32 (4 B)
-> COBS encode
-> append 0x00 delimiter
-> UART 921600 8N1
```

The Phase-8 compile-time tests pin important ABI details:

- `HeartbeatPayload` = 12 bytes
- `CommandPayload` = 8 bytes
- `CommandAckPayload` = 28 bytes
- `GnssNavV2Payload` = 68 bytes
- `Heartbeat` type = 32
- `Stop` type = 36
- `GnssNavV2` type = 59

The simulator uses actual production message types and payloads:

- communication -> control: `GnssNavV2`, gated `Heartbeat`, and `Arm` / `Disarm` / `StartTest` / `Stop` / `Estop` / `ClearEstop` commands using `CommandPayload`
- control -> communication: `ControlOutput`, `Heartbeat`, and `CommandAck`

`GnssNavV2Payload` is checked twice, like production software would require: the outer wire frame must pass COBS + frame CRC32, and the payload's canonical CRC must also match.

#### Production heartbeat chain

The production communication firmware does not blindly send a host heartbeat forever. It sends a 100 ms heartbeat only while it has received a valid control-side frame within the previous 1000 ms. The control side uses a 500 ms host-heartbeat/failsafe timeout.

Phase 8 models that chain:

```text
CONTROL -> COMM direction fails
        |
        v
COMM receives no fresh control frames for >1000 ms
        |
        v
COMM stops host heartbeat
        |
        v
CONTROL host heartbeat age exceeds 500 ms
        |
        v
CONTROL performs a local failsafe stop
```

This lets CI distinguish a single corrupt data frame from a true bidirectional supervision failure.

#### Production safety commands

Phase 8 sends real `CommandPayload` objects wrapped in the corresponding production type. The control-side model sends a real `CommandAckPayload` response.

The safety scenario exercises:

```text
DISARMED -> ARM -> START/RUNNING -> STOP/DISARMED
                                  -> E-STOP -> CLEAR_ESTOP/DISARMED
```

STOP and heartbeat-failsafe are intentionally separate test paths.

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

./build/production_protocol_sim scenarios/production_protocol_faults.csv artifacts/production_protocol_faults
./build/production_protocol_sim scenarios/production_protocol_safety.csv artifacts/production_protocol_safety
```

Every simulator emits SVG state frames plus `trace.csv`.

## Scenario formats

The single-controller simulator preserves all older formats:

- Phase 1: 7 columns
- Phase 2: 12 columns
- Phase 3: 20 columns
- Phase 4: 27 columns
- Phase 5: 39 columns
- Phase 6: 49 columns

The abstract Phase-7 dual-controller simulator uses:

```text
t_ms,c2c_connected,c2m_connected,c2c_latency_ms,c2m_latency_ms,c2c_fault,c2m_fault,gnss_valid,send_stop
```

The production Phase-8 simulator uses:

```text
t_ms,c2c_connected,c2c_latency_ms,c2m_connected,c2m_latency_ms,c2c_fault,c2m_fault,gnss_valid,arm,start,stop,estop,clear_estop,disarm
```

Fault codes are `0=none`, `1=drop`, `2=corrupt`, and `3=framing`.

## Build the real CoreS3 firmware

```bash
pio run -e m5stack-cores3
```

The production protocol classes are host-simulator components in this repository. The existing CoreS3 Arduino firmware still builds independently with its physical hardware path.

## GitHub Actions

Every push and pull request runs two independent jobs.

1. **Host simulation**
   - builds app, device, abstract UART, and production-protocol models,
   - runs all application/integration tests plus Phase 3-8 tests,
   - executes six device scenarios, two Phase-7 UART scenarios, and two Phase-8 production-protocol scenarios,
   - uploads SVG frames and trace CSV files.
2. **CoreS3 firmware build**
   - installs PlatformIO,
   - compiles the real M5Stack CoreS3 Arduino firmware.

## Important limitation

This is not an electrical, optical, RF, or instruction-level emulator. It does not reproduce UART waveform edges, oscillator mismatch, metastability, ESP32-S3 DMA/interrupt races, I2C analog effects, VL53L5CX optical physics, RF behavior, or brownouts.

Phase 8 is **protocol-core accurate**, not yet a native build of the complete production communication/control applications. The control-side model currently emits representative production `ControlOutput` + `Heartbeat` traffic rather than the complete real telemetry burst (`ControlSnapshot`, `InaStatus`, `VescTelemetry`, `ActuatorState`, `SystemHealth`, etc.).

## Next useful extension

The next step is to import the real command-ingress/ACK/retry and production telemetry schedule, then replay real hardware logs against the same decoder and compare host simulation with captured hardware traces.
