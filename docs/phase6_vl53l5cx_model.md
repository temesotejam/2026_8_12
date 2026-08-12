# Phase 6 VL53L5CX model notes

The Phase-6 simulator models the software-visible behavior of an 8x8 VL53L5CX ranging stream rather than electrical/optical physics.

- 64 zones are updated together as one frame.
- The 8x8 simulator caps ranging frequency at 15 Hz.
- Each zone carries both a distance and a target-status code.
- Statuses 5, 6, 9, and 12 are treated as usable by this proof-of-concept application policy.
- A zone is also rejected when its distance is outside the modeled 2..4000 mm range.
- The frame becomes stale after the larger of 200 ms or three configured frame periods.
- Device power loss, device-specific NACK, reset, I2C loss, ranging stop, and frame-stream stall can be injected independently.
- Device recovery uses the same host-side reinitialization/retry pattern as the BNO08X and INA226 models.

The model exposes valid-zone count and minimum valid distance to the application/UI. It does not attempt to emulate SPAD histograms, optical crosstalk, ambient-light physics, multi-target histogram internals, or exact firmware boot timing.
