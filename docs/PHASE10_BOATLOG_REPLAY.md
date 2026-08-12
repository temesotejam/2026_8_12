# Phase 10: BOATLOG1 hardware-log replay and comparison

Phase 10 adds a deterministic parser/comparator for the compact SD log format currently being developed for the boat communication XIAO.

## Log-format provenance

The format is taken from the currently open draft PR in the production boat repository:

- repository: `temesotejam/2026_8_6`
- PR: `#2 Add boat control web UI and sensor monitoring`
- PR head used for Phase-10 format inspection: `0820d1d1c2107609c839bf6cc5d8c42f9d56e982`
- base production revision: `bf8067c35ee56c884bfbd16e277b27db6e72ef98`
- defining file: `communication/src/main.cpp`

Because PR #2 is still draft, the BOATLOG1 format is treated as **draft format version 1**, not as a permanently stable production ABI. The parser rejects unknown versions or record sizes so a later firmware format change fails visibly instead of being silently mis-decoded.

## BOATLOG1 binary ABI

The draft firmware defines a 20-byte little-endian header:

```text
magic[8] = "BOATLOG1"
uint16 version = 1
uint16 recordBytes = 40
uint32 bootId
uint32 startMillis
```

Each fixed record is exactly 40 bytes:

```text
uint32 uptimeMs
int32  latitudeE7
int32  longitudeE7
int32  vescErpm
int16  speedCmPerS
int16  rollMilliRad
int16  pitchMilliRad
int16  tofMm
int16  leftMilli
int16  rightMilli
int16  rearMilli
int16  targetDutyTenThousandth
int16  appliedDutyTenThousandth
uint16 waypointDistanceCm
uint8  safety
uint8  mode
uint8  yawCriteria
uint8  flags
```

The draft logger currently uses a 2000 ms logging interval and flushes each record. A power loss can therefore leave trailing partial bytes after the last complete record. Phase 10 preserves all complete records and reports `trailing_bytes` rather than discarding the whole log.

## Normalized replay fields

Phase 10 converts each binary record to engineering units:

- uptime in ms
- latitude/longitude in degrees
- ground speed in m/s
- roll/pitch in rad
- ToF distance in m
- left/right/rear normalized commands
- target/applied duty
- waypoint distance in m
- VESC ERPM
- safety state
- control mode
- yaw-criterion bit mask
- validity/status flags

The decoded values can be written to CSV for inspection.

## Hardware-versus-host comparison

`boatlog_replay_cli` aligns each observed hardware sample with the nearest host-reference sample by `uptime_ms`. The default maximum timestamp offset is 150 ms.

It reports mean and maximum error for:

- timestamp alignment
- GNSS position in metres
- ground speed
- roll
- pitch
- ToF distance
- left/right/rear commands
- target duty
- applied duty
- waypoint distance
- VESC ERPM

It also counts exact mismatches in:

- safety
- mode
- yaw criteria
- flags

The default thresholds are intentionally explicit in `CompareThresholds`; they are a starting point for simulator-validation work, not claims about final real-world accuracy.

## CI fixture strategy

Until a real `BOAT_*.BIN` file is committed or uploaded for a test run, CI creates a synthetic hardware/reference pair using the **imported production `production_control::Controller`** from Phase 9.

The fixture generator creates four BOATLOG1 files:

1. `reference.bin` — host-model reference.
2. `observed_exact.bin` — identical observation; comparison must PASS.
3. `observed_shifted_80ms.bin` — all timestamps shifted by 80 ms; nearest-time alignment must PASS.
4. `observed_perturbed.bin` — deliberate position, wing, duty, safety, and ERPM differences; comparison must FAIL and CI explicitly checks that `summary.json` contains `"pass": false`.

This prevents a comparator bug that simply reports PASS for everything.

## Output artifacts

A comparison run writes:

```text
observed_decoded.csv
reference_decoded.csv
comparison.csv
summary.json
summary.svg
```

The JSON is suitable for machine gating; CSV is intended for detailed engineering inspection; SVG gives a quick artifact preview.

## Accuracy boundary

Phase 10 compares **what BOATLOG1 actually records**. The current 40-byte log does not contain every control input required to rerun the complete production controller from first principles: for example it does not store every raw IMU sample, all angular rates, power/current detail, command/retry history, waypoint definitions, or the complete UART timeline.

Therefore Phase 10 currently answers:

> For the observables that the hardware logged, how closely does the hardware trace match a host-simulation reference at corresponding times?

It does **not yet** answer:

> Can the full production firmware be deterministically reconstructed from only this 40-byte record stream?

To reach full deterministic input replay, a future log format should include or correlate command events, complete controller-relevant sensor state, tuning/configuration, and timing metadata. The Phase-10 parser/comparator is intentionally structured so those fields can be added in later log versions.

## Future real-log workflow

When a real SD log is available:

```bash
./build/boatlog_replay_cli \
  BOAT_XXXXXXXX_01.BIN \
  host_reference.bin \
  artifacts/real_run_01
```

The real hardware file does not need to be committed permanently; it can also be supplied to a local run or CI job as an external test artifact.
