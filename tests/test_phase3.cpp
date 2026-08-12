#include "App.h"
#include "VirtualHardware.h"

#include <cstdlib>
#include <iostream>
#include <string>

using namespace cores3sim;

namespace {
void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

bool has(const std::string& s, const char* part) {
  return s.find(part) != std::string::npos;
}

UiFrame step(App& app, VirtualHardware& hw, std::uint32_t t,
             const VirtualHwInput& in, const TouchSample& touch = {}) {
  hw.apply(t, in);
  return app.update(t, hw.sample(t), touch);
}
}

int main() {
  App app;
  VirtualHardware hw;
  VirtualHwInput in;
  in.i2c_latency_ms = 2;
  in.uart_latency_ms = 0;

  require(step(app, hw, 0, in).state == RunState::Boot, "phase3 boot");
  require(step(app, hw, 500, in).state == RunState::Ready, "phase3 ready");

  TouchSample press{true, 160, 205};
  require(step(app, hw, 600, in, press).state == RunState::Running,
          "phase3 start");
  step(app, hw, 650, in);

  in.i2c_nack = true;
  auto nack = step(app, hw, 700, in);
  require(nack.state == RunState::Fault, "I2C NACK must force FAULT");
  require(nack.message == "I2C device NACK", "NACK must be identified");
  require(hw.stats().i2c_nacks == 1, "NACK counter must increment");

  in.i2c_nack = false;
  step(app, hw, 800, in);
  require(step(app, hw, 1300, in).state == RunState::Ready,
          "I2C NACK must recover after 500 ms healthy");
  step(app, hw, 1350, in);
  require(step(app, hw, 1400, in, press).state == RunState::Running,
          "control must restart after NACK recovery");
  step(app, hw, 1450, in);

  in.uart_corrupt_byte = true;
  auto corrupt = step(app, hw, 1500, in);
  require(corrupt.state == RunState::Running,
          "UART byte corruption is warning-only");
  require(has(corrupt.warning, "UART frame error"),
          "corrupt UART frame must be visible");
  require(hw.stats().uart_corrupt_bytes == 1, "corrupt-byte counter");
  require(hw.stats().uart_crc_errors == 1,
          "byte corruption must be rejected by checksum");

  in.uart_corrupt_byte = false;
  in.uart_crc_error = true;
  auto crc = step(app, hw, 1600, in);
  require(has(crc.warning, "UART frame error"), "CRC warning");
  require(hw.stats().uart_crc_errors == 2, "explicit CRC counter");

  in.uart_crc_error = false;
  in.uart_framing_error = true;
  auto framing = step(app, hw, 1700, in);
  require(has(framing.warning, "UART frame error"), "framing warning");
  require(hw.stats().uart_framing_errors == 1, "framing-error counter");

  in.uart_framing_error = false;
  in.gnss_source_valid = false;
  in.loop_jitter_ms = 35;
  auto jitter = step(app, hw, 1800, in);
  require(jitter.state == RunState::Running, "jitter is warning-only");
  require(has(jitter.warning, "Control loop jitter"), "jitter warning");
  require(hw.stats().timing_jitter_events == 1, "jitter event counter");
  require(hw.stats().max_abs_jitter_ms == 35, "maximum jitter tracking");

  in.loop_jitter_ms = 0;
  in.sd_fail_write = true;
  auto sd = step(app, hw, 1900, in);
  require(sd.state == RunState::Running, "SD failure is warning-only");
  require(has(sd.warning, "SD write failed"), "SD failure warning");
  require(hw.stats().sd_write_failures >= 1, "SD failure counter");

  in.sd_fail_write = false;
  in.sd_latency_ms = VirtualHardware::kSdTimeoutMs + 1;
  auto sd_timeout = step(app, hw, 2000, in);
  require(has(sd_timeout.warning, "SD write failed"), "SD timeout warning");
  require(hw.stats().sd_write_timeouts == 1, "SD timeout counter");

  in.sd_latency_ms = 1;
  in.loop_jitter_ms = -30;
  in.uart_corrupt_byte = true;
  in.gnss_source_valid = true;
  in.sd_fail_write = true;
  auto combined = step(app, hw, 2100, in);
  require(has(combined.warning, "UART frame error"), "combined UART warning");
  require(has(combined.warning, "Control loop jitter"), "combined timing warning");
  require(has(combined.warning, "SD write failed"), "combined SD warning");

  std::cout << "All phase-3 fault tests passed.\n";
  return 0;
}
