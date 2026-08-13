# Phase 11: BOATLOG2 deterministic controller replay

Phase 10 compares observables in the compact 40-byte `BOATLOG1` SD record. That is useful for hardware-vs-host validation, but it cannot reconstruct `production_control::Controller` deterministically because the file does not contain all controller inputs or state-changing command events.

Phase 11 defines `BOATLOG2`, a deterministic replay stream for the production controller used by this repository.

## Design goals

- Re-run the imported production `production_control::Controller` one event at a time on a desktop host.
- Capture every external event that can change controller state: reset, config, mode, manual, heading, waypoint list, waypoint reach radius, and sensor-step input.
- Store the expected command result or controller output next to the event that produced it.
- Use stable packed wire structures rather than serializing C++ structs with compiler-dependent padding or `bool` representation.
- Detect corrupted records with a CRC32 and sequence number.
- Preserve all complete records if power is lost during a final record write.
- Fail closed when the format version, fixed record size, endian marker, controller ABI ID, record CRC, event ordering, or expected output disagrees.

## File format

`BOATLOG2` is little-endian and uses a 32-byte header followed by fixed 320-byte records.

Header:

```text
magic[8]             = "BOATLOG2"
version              = 2
header_bytes         = 32
record_bytes         = 320
flags                = 0
controller_abi_id    = 0xBF8067C3
boot_id              = producer-defined
start_us              = producer-defined
reserved             = 0
```

`controller_abi_id` deliberately pins this Phase-11 reader to the production-controller source lineage imported by Phase 9 (`temesotejam/2026_8_6` commit `bf8067c35ee56c884bfbd16e277b27db6e72ef98`). If the production controller changes, a new ABI ID should be chosen rather than silently replaying old captures against changed logic.

Each fixed record is:

```text
type                 uint8
flags                uint8
payload_bytes        uint16
sequence             uint32
at_us                uint64
payload              300 bytes
record_crc32         uint32
```

CRC32 is calculated over the first 316 bytes of the record using the same reflected `0xEDB88320` CRC implementation already used by `boat_protocol`.

## Record types

- `Reset`: calls `Controller::reset()`.
- `SetConfig`: calls `Controller::setConfig()` with every production-control setting represented explicitly.
- `SetMode`: records command arguments plus expected `CommandResult`.
- `SetManual`: records the complete manual command, receive timestamp, and expected `CommandResult`.
- `SetHeading`: records target heading, request ID, and expected `CommandResult`.
- `SetWaypoints`: records up to 16 north/east points, request ID, safety state, and expected `CommandResult`.
- `SetWaypointReachRadius`: records radius, safety state, and expected `CommandResult`.
- `Step`: records every `SensorInput` field and the complete expected `Output` returned by `Controller::step()`.

The replay state is reconstructed by processing the file strictly from the first record to the last record. Internal controller fields are not serialized; they are rebuilt by applying the same external events in the same order.

## Output comparison

Categorical/state outputs must match exactly:

- safety
- mode
- stop reason
- safety request
- flags
- active waypoint
- enabled output mask
- saturated
- physical gate
- waypoint reached
- command ACK and reason

Float outputs are compared against a small absolute tolerance (default `1e-5`) to allow the same float32 values to be evaluated on ESP32 and host FPUs without requiring identical intermediate instruction sequences.

The replay report includes:

- total/processed records
- CRC and sequence failures
- malformed/unknown record count
- command-result mismatches
- step-output mismatches
- maximum float error
- first failing record and field
- pass/fail

## Power-loss behavior

A trailing partial record is ignored after all complete records have been validated and is reported as `trailing_bytes`. A CRC-bad complete record is **not** ignored: replay fails because that record may contain a corrupted controller input.

## Scope boundary

This is deterministic replay of `production_control::Controller`, not of the complete two-XIAO firmware. It does not reconstruct FreeRTOS scheduling, UART queues, device drivers, sensor algorithms, Web UI, or physical actuator timing. The important improvement over BOATLOG1 is that every external input and state-changing event required by `Controller` is now present.

A later production-firmware instrumentation step can write the same packed header/records around the real `Controller` call site. Once that exists, a real hardware `BOATLOG2` file can be fed directly to the host replay CLI without conversion.
