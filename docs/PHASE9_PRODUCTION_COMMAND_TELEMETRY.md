# Phase 9: production command ingress, retry, and telemetry

Phase 9 moves the host simulation beyond the Phase-8 wire codec and into the current production command/control contract used by the two-XIAO boat firmware.

## Pinned production source

Source repository: `temesotejam/2026_8_6`

Pinned revision: `bf8067c35ee56c884bfbd16e277b27db6e72ef98`

The following production sources are compiled on the host in Phase 9:

| Production source | Phase-9 copy | Source blob SHA |
|---|---|---|
| `control/lib/production_control/src/production_control.h` | `lib/production_control/production_control.h` | `83837bca` (pinned source) |
| `control/lib/production_control/src/production_control.cpp` | `lib/production_control/production_control.cpp` | `dd8bd92c` (pinned source) |
| `control/lib/production_control/src/command_replay.h` | `lib/production_control/command_replay.h` | `7cf65e0` (pinned source) |
| `control/lib/production_control/src/command_replay.cpp` | `lib/production_control/command_replay.cpp` | `06df49f` (pinned source) |
| `control/lib/production_control/src/command_ingress.h` | `lib/production_control/command_ingress.h` | `dad1b09` (pinned source) |
| `control/lib/production_control/src/command_ingress.cpp` | `lib/production_control/command_ingress.cpp` | `5c6c7e3` (pinned source) |

The short hashes above are recorded as provenance labels; the pinned Git commit is the authoritative source revision. Phase 8 already records the exact `boat_protocol.h/.cpp` source blobs separately in `docs/PRODUCTION_PROTOCOL_SOURCE.md`.

## What is compiled directly from the production implementation

Phase 9 uses the copied production implementations for:

- `production_control::Controller`
- `Controller::step()` safety/control logic
- `ControlModeCommand`, `ManualCommand`, and `HeadingTarget` application
- canonical command CRC validation
- range/protocol-version validation
- `CommandReplayWindow`
- 64-entry replay history
- wrap-aware command sequence ordering
- duplicate detection
- request/sequence conflict detection
- stale sequence detection
- `CommandIngress` ACK disposition/reason behavior

This code is linked against the Phase-8 production `boat_protocol` host library, so the command ingress sees the same packed payloads produced by the same COBS/CRC32 wire codec.

## COMM-side PendingCommand behavior modeled from production

The communication firmware currently uses:

- retry interval: **100 ms**
- command timeout: **1200 ms**
- maximum tracked retry attempts: **8**

`ProductionCommandTelemetrySystem` reproduces that policy. A pending command keeps the same *inner* `requestId` / `commandSequence` (or Waypoint revision) across retries, while each UART frame itself gets a new outer protocol frame sequence.

That distinction is important: if the CONTROL side applies a command but the ACK is lost, the same command arrives again and the production 64-entry replay window returns `Duplicate` instead of applying the command twice.

The Phase-9 retry scenario deliberately drops the first `ControlCommandAck` and the first `WaypointAck` to validate both duplicate paths.

## Production malformed-command behavior preserved

The current `CommandIngress` comments and implementation intentionally do **not** replay-store syntax failures. A command with a bad canonical CRC is classified as malformed before it enters the replay window.

The current production malformed-ACK path does not preserve the original request/sequence identifiers used by COMM PendingCommand matching. Therefore the Phase-9 host model preserves the current observable behavior: COMM continues retrying and eventually reaches the 1200 ms pending-command timeout.

This is intentionally tested rather than silently “fixing” the production behavior inside the simulator.

## Manual command freshness

The production communication application does two different kinds of resend:

1. **Pending retry** — same request/command IDs, used when an ACK is missing.
2. **Manual refresh** — new request/command IDs every **200 ms**, used to keep manual input fresh during non-waypoint operation.

Phase 9 keeps these mechanisms separate. `ProductionCommandTelemetryManualRefresh.cpp` implements the 200 ms COMM-side refresh adapter. The production `CommandIngress` sees every refresh as a new command, not as a replay duplicate.

The Phase-9 C++ suite runs manual mode for more than one second and asserts that the 200 ms refresh prevents the production controller's 500 ms manual timeout. STOP/Disarm/E-stop clear the refresh stream, and a later manual stream must begin with a new accepted `SetManual` command.

## Production telemetry burst

The pinned CONTROL firmware's `runProductionControl()` emits a production telemetry bundle every **100 ms** after running the production controller. The observed order in `controller_service.inc` is:

1. `ControlOutput`
2. `ControlSnapshot`
3. `InaStatus`
4. `VescTelemetry`
5. `ActuatorState`
6. `SystemHealth`

Phase 9 sends the same six packed production payload types in that order at the same 100 ms cadence. COMM decodes and caches all six with the production `boat::Decoder`.

The controller output is produced by the imported production `Controller::step()` using representative virtual sensor/power/motor inputs. Sensor values are simulated; the control algorithm and payload ABI are production-derived.

## Bidirectional supervision

The full telemetry burst counts as CONTROL->COMM link activity. The Phase-8 heartbeat chain remains in force:

- COMM considers CONTROL fresh for 1000 ms after receiving a valid CONTROL frame.
- COMM sends host heartbeat every 100 ms only while that reverse link is fresh.
- CONTROL treats host heartbeat older than 500 ms as a local fail-safe condition while active.

The Phase-9 telemetry-failsafe scenario disconnects CONTROL->COMM while leaving COMM->CONTROL electrically available. This proves that losing the *feedback/telemetry direction* eventually suppresses COMM heartbeat and forces the CONTROL node to self-stop.

## Phase-9 scenarios

- `production_command_retry.csv`
  - configure AutoWaypoint mode
  - drop first mode ACK
  - retry with same inner IDs
  - production replay returns Duplicate
  - set waypoint
  - drop first WaypointAck
  - retry returns waypoint Duplicate/Revision ACK
  - ARM -> START -> RUNNING
  - full telemetry continues
  - STOP -> DISARMED

- `production_command_timeout.csv`
  - inject a bad ManualCommand canonical CRC
  - observe repeated malformed classification
  - COMM pending retry continues
  - pending command reaches the current 1200 ms timeout behavior

- `production_telemetry_failsafe.csv`
  - configure AutoWaypoint + waypoint
  - ARM -> START -> RUNNING
  - disconnect CONTROL->COMM
  - full telemetry disappears
  - COMM link becomes stale
  - COMM host heartbeat stops
  - CONTROL heartbeat fail-safe self-stops

## Accuracy boundary

Phase 9 is still a native behavior/protocol simulation, not a complete ESP32/XIAO emulator.

Production sources compiled directly:

- boat wire codec/ABI
- production controller
- command replay window
- command ingress

Host adapters modeled from production behavior:

- COMM PendingCommand scheduling
- manual 200 ms refresh scheduling
- safety-operation orchestration
- production telemetry scheduler
- representative sensor/power/VESC values
- virtual UART timing/fault injection

It does not reproduce FreeRTOS task scheduling, HardwareSerial buffer details, UART DMA/interrupt races, electrical UART waveforms, real sensor physics, RF, or power-domain behavior.
