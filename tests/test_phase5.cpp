#include "App.h"
#include "VirtualHardware.h"

#include <cmath>
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
  in.ina_model_enabled = true;
  in.ina_bus_voltage_v = 12.0f;
  in.ina_current_a = 2.0f;
  in.ina_shunt_ohm = 0.002f;
  in.ina_current_lsb_a = 0.00125f;
  in.ina_shunt_conversion_us = 588;
  in.ina_bus_conversion_us = 588;
  in.ina_averages = 16;
  in.ina_reinit_delay_ms = 20;

  auto init = step(app, hw, 0, in);
  require(init.ina_reinitializing, "INA226 must start host initialization");
  require(has(init.warning, "INA226 init"), "INA init must be visible");

  auto configured = step(app, hw, 20, in);
  require(configured.ina_initialized, "INA226 init must complete");
  require(!configured.ina_conversion_fresh,
          "no conversion should exist at init completion");

  auto first = step(app, hw, 40, in);
  require(first.ina_conversion_fresh,
          "first averaged conversion must complete");
  require(std::fabs(first.ina_bus_voltage_v - 12.0f) < 0.01f,
          "bus voltage conversion");
  require(std::fabs(first.ina_current_a - 2.0f) < 0.01f,
          "current conversion");
  require(std::fabs(first.ina_power_w - 24.0f) < 0.1f,
          "power conversion");

  require(step(app, hw, 500, in).state == RunState::Ready,
          "app ready with INA healthy");
  TouchSample press{true, 160, 205};
  require(step(app, hw, 600, in, press).state == RunState::Running,
          "start control");
  step(app, hw, 605, in);

  in.ina_calibration_programmed = false;
  auto uncal = step(app, hw, 620, in);
  require(uncal.state == RunState::Running,
          "INA calibration loss is warning-only");
  require(!uncal.ina_calibration_ok, "calibration flag");
  require(std::fabs(uncal.ina_current_a) < 0.0001f,
          "current register must be zero without calibration");
  require(std::fabs(uncal.ina_power_w) < 0.0001f,
          "power register must be zero without calibration");
  require(has(uncal.warning, "INA226 uncalibrated"),
          "uncalibrated warning");

  in.ina_calibration_programmed = true;
  auto recal = step(app, hw, 640, in);
  require(recal.ina_calibration_ok, "calibration restored");
  require(recal.ina_current_a > 1.9f,
          "current must recover after calibration");

  in.ina_stall_conversions = true;
  step(app, hw, 660, in);
  auto stale = step(app, hw, 760, in);
  require(stale.state == RunState::Running,
          "INA stale is warning-only");
  require(!stale.ina_conversion_fresh,
          "stalled INA conversions must become stale");
  require(has(stale.warning, "INA226 data stale"), "stale warning");

  in.ina_stall_conversions = false;
  auto resumed = step(app, hw, 780, in);
  require(resumed.ina_conversion_fresh,
          "conversion stream must recover");

  in.ina_device_ack = false;
  auto offline = step(app, hw, 800, in);
  require(!offline.ina_device_ok, "device NACK/offline must be visible");
  require(has(offline.warning, "INA226 offline"), "offline warning");
  require(offline.state == RunState::Running,
          "INA device loss is warning-only");
  require(hw.inaStats().device_nacks == 1,
          "INA device NACK counter");

  in.ina_device_ack = true;
  auto reinit = step(app, hw, 820, in);
  require(reinit.ina_reinitializing,
          "INA must auto-reinitialize after device return");
  auto reinit_done = step(app, hw, 840, in);
  require(reinit_done.ina_initialized, "INA reinit complete");
  require(!reinit_done.ina_conversion_fresh,
          "conversion must wait after reinit");
  auto reinit_data = step(app, hw, 860, in);
  require(reinit_data.ina_conversion_fresh,
          "data must recover after reinit");

  in.ina_bus_voltage_v = 40.0f;
  auto range = step(app, hw, 880, in);
  require(!range.ina_range_ok,
          "bus voltage beyond 36V must be range error");
  require(has(range.warning, "INA226 input range"), "range warning");

  in.ina_bus_voltage_v = 12.0f;
  in.ina_force_math_overflow = true;
  auto ovf = step(app, hw, 900, in);
  require(ovf.ina_math_overflow, "math overflow flag");
  require(has(ovf.warning, "INA226 math overflow"),
          "overflow warning");

  in.ina_force_math_overflow = false;
  in.ina_force_reset = true;
  auto reset = step(app, hw, 920, in);
  require(reset.ina_reinitializing,
          "INA reset must trigger reconfiguration");
  require(hw.inaStats().resets == 1, "INA reset counter");

  in.ina_force_reset = false;
  step(app, hw, 940, in);
  auto recovered = step(app, hw, 960, in);
  require(recovered.ina_initialized && recovered.ina_conversion_fresh,
          "INA must recover after reset and new conversion");

  require(hw.inaStats().reinit_successes >= 3,
          "startup, device return, reset reinit successes");
  require(hw.inaStats().conversions_completed > 0,
          "conversion counter");
  require(hw.inaStats().stale_events >= 1, "stale counter");
  require(hw.inaStats().range_errors >= 1, "range error counter");
  require(hw.inaStats().math_overflows >= 1, "overflow counter");
  require(hw.inaStats().calibration_missing_conversions >= 1,
          "missing calibration counter");

  std::cout << "All phase-5 INA226 tests passed.\n";
  return 0;
}
