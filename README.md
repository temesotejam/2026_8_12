# CoreS3 Wokwi / GitHub Actions smoke test

This repository is a small proof of concept for running the **same M5Stack CoreS3 firmware** in Wokwi and checking its behavior from GitHub Actions.

The first goal is intentionally small: prove that a real PlatformIO firmware image for CoreS3 can boot in the simulator, initialize M5Unified/display code, execute the main loop, and emit an expected result over Serial.

## What is tested now

The firmware in `src/main.cpp`:

1. starts on the M5Stack CoreS3 target,
2. initializes M5Unified,
3. writes a message to the CoreS3 display,
4. prints display dimensions over Serial,
5. emits a heartbeat once per second,
6. prints `SIM_CHECK:PASS` after several successful loop iterations when the display object is available.

The GitHub Actions workflow always performs a PlatformIO build. When the repository secret `WOKWI_CLI_TOKEN` is configured, it also boots the compiled firmware in Wokwi and requires `SIM_CHECK:PASS` to appear. `SIM_CHECK:FAIL` fails the Wokwi step.

## Files

- `platformio.ini` - real CoreS3 PlatformIO build configuration
- `src/main.cpp` - minimal production-style firmware used by both real hardware and Wokwi
- `wokwi.toml` - points Wokwi at PlatformIO's compiled `.bin` and `.elf`
- `diagram.json` - simulated M5Stack CoreS3 board
- `.github/workflows/ci.yml` - build + optional Wokwi behavior test
- `docs/TEST_SCOPE.md` - what this test proves and what still requires real hardware

## Build locally

```bash
pio run
```

The target is the PlatformIO `m5stack-cores3` board.

## Run interactively with Wokwi

1. Install PlatformIO in VS Code.
2. Install the Wokwi VS Code extension.
3. Run a PlatformIO build once (`pio run`).
4. Run `Wokwi: Start Simulator` from the VS Code command palette.

Expected serial output includes lines similar to:

```text
BOOT:CORE_S3_SMOKE_V1
DISPLAY:320x240
HEARTBEAT:1
HEARTBEAT:2
HEARTBEAT:3
SIM_CHECK:PASS
```

The exact display dimensions line is informational; the smoke test only requires a non-zero display size and successful loop execution.

## Enable the GitHub Actions simulation

Create a Wokwi CI token and add it as this repository secret:

`WOKWI_CLI_TOKEN`

Without the secret, the ordinary PlatformIO build still runs and the Wokwi step reports that it was skipped. With the secret, Wokwi boots the generated CoreS3 firmware and checks the actual serial behavior.

## Why this layout matters

There is no separate "simulator firmware" here. The same `src/main.cpp` and the same compiled firmware intended for CoreS3 are used by the simulator. Later, virtual GNSS/IMU devices can be added around the CoreS3 without replacing the application logic.

## References

- Wokwi project configuration: https://docs.wokwi.com/vscode/project-config
- Wokwi GitHub Actions: https://docs.wokwi.com/wokwi-ci/github-actions
- PlatformIO CoreS3 board: https://docs.platformio.org/en/latest/boards/espressif32/m5stack-cores3.html
