# Phase 8 production protocol source

Phase 8 intentionally uses the boat UART protocol from the real two-XIAO boat firmware rather than a simulator-only packet definition.

## Source repository and pinned revision

- Repository: `temesotejam/2026_8_6`
- Pinned source commit: `bf8067c35ee56c884bfbd16e277b27db6e72ef98`
- Protocol header: `control/lib/boat_protocol/src/boat_protocol.h`
  - source blob SHA: `4fc4b7c35b27aa21983f80c10fd8ce038db04f8b`
- Protocol implementation: `control/lib/boat_protocol/src/boat_protocol.cpp`
  - source blob SHA: `9011bc67e8ecbca8a11fc57e7fb9d000d9c82470`

The communication-side firmware contains the same `boat_protocol` implementation at that revision. Phase 8 copies the protocol header and codec implementation into `lib/boat_protocol/` so the host simulator compiles and executes the same packed ABI, CRC32, COBS encoder, and streaming decoder.

## Production wire contract represented by Phase 8

- protocol version: `1`
- maximum payload: `768` bytes
- packed header size: `22` bytes
- raw frame: `Header + payload + uint32 CRC32`
- wire frame: COBS-encoded raw frame followed by `0x00`
- CRC32: reflected polynomial `0xEDB88320`, initial `0xFFFFFFFF`, final inversion
- UART: `921600` baud, `8N1`

Important production message IDs exercised in the Phase-8 simulator:

- `Heartbeat = 32`
- `Arm = 33`
- `Disarm = 34`
- `StartTest = 35`
- `Stop = 36`
- `Estop = 37`
- `ClearEstop = 38`
- `GnssNavV2 = 59`
- `ControlOutput = 62`
- `CommandAck = 17`

Important payload ABIs checked by compile-time tests:

- `HeartbeatPayload`: 12 bytes
- `CommandPayload`: 8 bytes
- `CommandAckPayload`: 28 bytes
- `GnssNavV2Payload`: 68 bytes

`GnssNavV2Payload` also contains its production canonical payload CRC. Phase 8 validates that CRC separately after the outer COBS/CRC32 frame has decoded.

## Production timing and heartbeat policy represented by Phase 8

The pinned control firmware config uses:

- link baud: 921600
- communication GNSS_NAV interval: 100 ms
- control heartbeat interval: 100 ms
- host heartbeat/failsafe timeout: 500 ms

The communication firmware uses a 100 ms heartbeat period but only sends its host heartbeat while it has received a valid control-side frame within the previous 1000 ms. Phase 8 models this bidirectional supervision chain. Therefore loss of the control-to-communication direction eventually stops communication-side heartbeat transmission, which then causes the control side to stop locally after the 500 ms host-heartbeat timeout.

## Intentional Phase-8 scope limit

Phase 8 is protocol-core accurate, not yet a full native build of both production applications. The control side currently emits a representative production `ControlOutput` plus production heartbeat rather than the complete production telemetry burst (`ControlSnapshot`, `InaStatus`, `VescTelemetry`, `ActuatorState`, `SystemHealth`, etc.).

The next stage can import the real command ingress / telemetry scheduling and then replay hardware logs against the host simulation.
