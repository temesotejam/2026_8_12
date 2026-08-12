#include "App.h"
#include "VirtualHardware.h"
#include "VirtualVl53l5cx.h"

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

bool has(const std::string& text, const char* part) {
  return text.find(part) != std::string::npos;
}

UiFrame step(App& app, VirtualHardware& hw, std::uint32_t t,
             const VirtualHwInput& in, const TouchSample& touch = {}) {
  hw.apply(t, in);
  return app.update(t, hw.sample(t), touch);
}
}

int main() {
  VirtualVl53l5cx tof;
  Vl53l5cxInput in;
  in.enabled = true;
  in.ranging_frequency_hz = 10;
  in.reinit_delay_ms = 300;
  in.default_distance_mm = 800;

  auto s0 = tof.update(0, in, true);
  require(s0.reinitializing && !s0.initialized, "startup reinit must begin");
  require(tof.update(299, in, true).reinitializing,
          "must remain in init before delay");

  auto s300 = tof.update(300, in, true);
  require(s300.initialized && s300.frame_fresh,
          "first frame should be available after init");
  require(s300.valid_zones == 64 && s300.min_distance_mm == 800,
          "all 64 zones should be valid in normal frame");

  in.invalid_zone = 27;
  in.invalid_status = 255;
  in.invalid_distance_mm = 0;
  auto s400 = tof.update(400, in, true);
  require(s400.valid_zones == 63, "one invalid zone must reduce valid count");
  require(!s400.zone_valid[27], "invalid zone must be flagged");

  in.invalid_zone = 255;
  in.stall_frames = true;
  require(tof.update(600, in, true).frame_fresh,
          "frame should remain fresh before stale threshold");
  auto s701 = tof.update(701, in, true);
  require(!s701.frame_fresh, "stalled frame must become stale");
  require(tof.stats().stale_events == 1, "stale event counter");

  in.stall_frames = false;
  auto s800 = tof.update(800, in, true);
  require(s800.frame_fresh && s800.valid_zones == 64,
          "frame stream must recover");

  in.device_ack = false;
  auto nack = tof.update(900, in, true);
  require(!nack.initialized && !nack.frame_fresh,
          "device NACK must drop device state");
  require(tof.stats().device_nacks == 1, "device NACK counter");

  in.device_ack = true;
  require(tof.update(1000, in, true).reinitializing,
          "ACK recovery must start reinit");
  require(tof.update(1300, in, true).frame_fresh,
          "device must recover after reinit");

  in.force_reset = true;
  auto reset = tof.update(1400, in, true);
  require(reset.reinitializing && !reset.initialized,
          "reset must trigger reinitialization");
  require(tof.stats().resets == 1, "reset counter");

  in.force_reset = false;
  require(tof.update(1700, in, true).frame_fresh,
          "reset recovery must deliver a frame");

  in.force_reset = true;
  tof.update(1800, in, true);
  in.force_reset = false;
  auto bus_down = tof.update(1900, in, false);
  require(!bus_down.initialized && !bus_down.reinitializing,
          "I2C loss during recovery must cancel init");
  require(tof.update(2000, in, true).reinitializing,
          "bus recovery must retry init");
  require(tof.update(2300, in, true).frame_fresh,
          "bus recovery must restore ranging");

  // Application-level policy: a VL53L5CX-only failure is warning-only,
  // while a shared I2C failure remains critical through the control IMU path.
  App app;
  VirtualHardware hw;
  VirtualHwInput vi;
  vi.tof_model_enabled = true;
  vi.tof_reinit_delay_ms = 100;
  vi.tof_ranging_frequency_hz = 10;
  vi.tof_default_distance_mm = 700;

  require(step(app, hw, 0, vi).state == RunState::Boot, "integration boot");
  require(step(app, hw, 500, vi).state == RunState::Ready,
          "ToF startup must not block ready once control sensors are healthy");

  TouchSample press{true, 160, 205};
  require(step(app, hw, 600, vi, press).state == RunState::Running,
          "integration start");
  step(app, hw, 650, vi);

  vi.tof_invalid_zone = 12;
  vi.tof_invalid_status = 255;
  auto partial = step(app, hw, 700, vi);
  require(partial.state == RunState::Running,
          "partial ToF zone failure must be warning-only");
  require(has(partial.warning, "VL53L5CX invalid zones"),
          "partial zone failure must be visible");

  vi.tof_invalid_zone = 255;
  vi.tof_stall_frames = true;
  step(app, hw, 900, vi);
  auto stale = step(app, hw, 1001, vi);
  require(stale.state == RunState::Running,
          "stale ToF frame must be warning-only");
  require(has(stale.warning, "VL53L5CX frame stale"),
          "stale ToF warning must be visible");

  vi.tof_stall_frames = false;
  vi.tof_device_ack = false;
  auto offline = step(app, hw, 1100, vi);
  require(offline.state == RunState::Running,
          "ToF-only NACK must not stop local control");
  require(has(offline.warning, "VL53L5CX offline"),
          "ToF NACK must be visible");

  vi.tof_device_ack = true;
  vi.i2c_connected = false;
  auto shared_bus_fault = step(app, hw, 1200, vi);
  require(shared_bus_fault.state == RunState::Fault,
          "shared I2C loss must remain critical through IMU path");

  std::cout << "All phase-6 VL53L5CX tests passed.\n";
  return 0;
}
