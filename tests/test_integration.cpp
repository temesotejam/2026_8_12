#include "App.h"
#include "VirtualHardware.h"

#include <cstdlib>
#include <iostream>

using cores3sim::App;
using cores3sim::RunState;
using cores3sim::TouchSample;
using cores3sim::VirtualHardware;
using cores3sim::VirtualHwInput;

namespace {
void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

cores3sim::UiFrame step(App& app,
                        VirtualHardware& hw,
                        std::uint32_t t,
                        const VirtualHwInput& in,
                        const TouchSample& touch = {}) {
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

  require(step(app, hw, 0, in).state == RunState::Boot, "integration boot");
  require(step(app, hw, 500, in).state == RunState::Ready, "integration ready");

  TouchSample press{true, 160, 205};
  require(step(app, hw, 600, in, press).state == RunState::Running,
          "integration start through virtual hardware");
  step(app, hw, 700, in);

  in.i2c_latency_ms = 25;
  auto fault = step(app, hw, 800, in);
  require(fault.state == RunState::Fault, "I2C timeout must propagate to app FAULT");

  in.i2c_latency_ms = 2;
  step(app, hw, 900, in);
  step(app, hw, 1200, in);
  require(step(app, hw, 1400, in).state == RunState::Ready,
          "healthy I2C must recover after 500 ms");

  in.uart_connected = false;
  step(app, hw, 1500, in);
  auto running = step(app, hw, 1600, in, press);
  require(running.state == RunState::Running,
          "UART outage must not block starting local control in demo policy");
  require(running.warning == "UART link down",
          "UART outage must remain visible while running");

  step(app, hw, 2200, in);
  auto stale = step(app, hw, 2201, in);
  require(!stale.gnss_ok, "GNSS must go stale during long UART outage");
  require(stale.state == RunState::Running,
          "stale GNSS is warning-only in this proof of concept");

  in.uart_connected = true;
  in.uart_latency_ms = 200;
  in.gnss_source_valid = true;
  step(app, hw, 2300, in);
  in.gnss_source_valid = false;
  step(app, hw, 2400, in);
  auto recovered = step(app, hw, 2500, in);
  require(recovered.uart_ok && recovered.gnss_ok,
          "GNSS must recover after delayed UART frame arrives");

  std::cout << "All integration tests passed.\n";
  return 0;
}
