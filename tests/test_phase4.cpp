#include "App.h"
#include "VirtualHardware.h"

#include <cstdlib>
#include <iostream>

using namespace cores3sim;

namespace {
void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
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
  in.bno_model_enabled = true;
  in.bno_report_interval_ms = 20;
  in.bno_reinit_delay_ms = 300;
  in.bno_auto_reinit = true;
  in.pitch_deg = 4.0f;

  auto boot0 = step(app, hw, 0, in);
  require(boot0.state == RunState::Boot,
          "BNO startup initialization must keep app in BOOT");
  require(boot0.bno_reinitializing,
          "BNO must report startup reinitialization");

  auto boot299 = step(app, hw, 299, in);
  require(boot299.state == RunState::Boot && boot299.bno_reinitializing,
          "BNO must still initialize before configured delay");

  auto initialized = step(app, hw, 300, in);
  require(initialized.state == RunState::Boot,
          "app remains BOOT until normal boot time");
  require(initialized.bno_initialized && initialized.bno_report_fresh,
          "BNO must initialize and publish first report");
  require(initialized.imu_ok, "fresh initialized BNO report must satisfy IMU");

  auto age10 = step(app, hw, 310, in);
  require(age10.bno_report_age_ms == 10,
          "BNO report age must advance between report intervals");

  auto report320 = step(app, hw, 320, in);
  require(report320.bno_report_age_ms == 0,
          "BNO report interval must produce a new report");

  require(step(app, hw, 500, in).state == RunState::Ready,
          "app must become READY with healthy BNO");

  TouchSample press{true, 160, 205};
  require(step(app, hw, 600, in, press).state == RunState::Running,
          "healthy BNO must allow RUNNING");
  step(app, hw, 650, in);

  in.bno_stall_reports = true;
  require(step(app, hw, 750, in).state == RunState::Running,
          "report age at stale threshold remains usable");
  auto stale = step(app, hw, 751, in);
  require(stale.state == RunState::Fault,
          "stale BNO report must force FAULT");
  require(stale.message == "BNO08X report stale",
          "stale BNO report must be identified");

  in.bno_stall_reports = false;
  step(app, hw, 800, in);
  require(step(app, hw, 1300, in).state == RunState::Ready,
          "fresh BNO reports must recover after 500 ms healthy");

  step(app, hw, 1350, in);
  require(step(app, hw, 1400, in, press).state == RunState::Running,
          "control must restart after BNO stale recovery");
  step(app, hw, 1450, in);

  in.bno_force_reset = true;
  auto resetting = step(app, hw, 1500, in);
  require(resetting.state == RunState::Fault,
          "BNO reset during RUNNING must force FAULT");
  require(resetting.message == "BNO08X reinitializing",
          "BNO reset must show reinitialization state");
  require(hw.bnoStats().resets == 1, "BNO reset counter must increment");

  step(app, hw, 1520, in);
  require(hw.bnoStats().resets == 1,
          "held reset signal must count only one reset edge");

  in.bno_force_reset = false;
  auto reinit_done = step(app, hw, 1800, in);
  require(reinit_done.bno_initialized && reinit_done.bno_report_fresh,
          "BNO must recover after configured reinit delay");
  require(reinit_done.state == RunState::Fault,
          "application fault recovery timer starts after BNO is healthy");
  require(step(app, hw, 2300, in).state == RunState::Ready,
          "application must return READY 500 ms after BNO recovery");

  step(app, hw, 2350, in);
  require(step(app, hw, 2400, in, press).state == RunState::Running,
          "BNO must support another run after reset recovery");
  step(app, hw, 2450, in);

  in.bno_force_reset = true;
  step(app, hw, 2500, in);
  in.bno_force_reset = false;
  in.i2c_connected = false;
  auto bus_down = step(app, hw, 2600, in);
  require(!bus_down.i2c_ok && bus_down.state == RunState::Fault,
          "I2C loss during BNO reinit must remain critical");

  in.i2c_connected = true;
  auto retry = step(app, hw, 2700, in);
  require(retry.bno_reinitializing,
          "BNO reinit must restart after I2C bus recovery");
  auto recovered = step(app, hw, 3000, in);
  require(recovered.bno_initialized && recovered.bno_report_fresh,
          "BNO must recover after retry");
  require(step(app, hw, 3500, in).state == RunState::Ready,
          "app must recover after retried BNO initialization");

  const auto& stats = hw.bnoStats();
  require(stats.resets == 2, "two BNO reset edges must be counted");
  require(stats.reinit_attempts == 4,
          "startup, reset, failed bus attempt, and retry must be counted");
  require(stats.reinit_successes == 3,
          "startup and two completed reinitializations expected");
  require(stats.stale_events >= 1,
          "stalled reports must increment stale-event counter");
  require(stats.reports_delivered >= 4,
          "BNO model must deliver multiple reports");

  std::cout << "All phase-4 BNO08X tests passed.\n";
  return 0;
}
