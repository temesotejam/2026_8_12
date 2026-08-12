# CoreS3 local simulation experiment

This repository is a proof-of-concept for checking M5Stack CoreS3 application behavior without depending on a hosted simulator for every test run.

## Phase 2: virtual buses

The simulator now puts a virtual hardware layer between scenarios and the shared application logic:

- virtual I2C + IMU with disconnect and latency/timeout injection,
- virtual UART + GNSS with disconnect, delivery latency, dropped frames, and stale-data detection,
- I2C/IMU faults are critical and force `FAULT`, while UART/GNSS loss is warning-only in this proof of concept,
- the original 7-column phase-1 scenarios remain supported.

The bus-aware scenario format is:

```text
t_ms,pitch_deg,battery_v,imu_online,i2c_connected,i2c_latency_ms,uart_connected,uart_latency_ms,gnss_valid,touch_pressed,touch_x,touch_y
```

`scenarios/bus_faults.csv` demonstrates an I2C timeout, recovery, UART disconnect, GNSS stale detection, delayed reconnect, and recovery.

## Run on a PC

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
./build/cores3_sim scenarios/demo.csv artifacts/demo
./build/cores3_sim scenarios/bus_faults.csv artifacts/bus_faults
```

The simulator writes SVG screen frames and `trace.csv`, including I2C/UART/GNSS health and transport error counters.

## Build the real CoreS3 firmware

```bash
pio run -e m5stack-cores3
```

GitHub Actions runs all host tests, both simulation scenarios, and a real CoreS3 PlatformIO compile on every push and pull request.

## Boundary of the simulation

This emulates the observable behavior of hardware interfaces (success, timeout, disconnect, delay, stale data, recovery), not their electrical physics. Real hardware is still needed for pull-up/bus-capacitance problems, line noise, interrupt/DMA races, power behavior, RF, and exact peripheral timing.
