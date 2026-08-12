#include "VirtualVl53l5cx.h"

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

  auto s299 = tof.update(299, in, true);
  require(s299.reinitializing, "must remain in init before delay");

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
  auto s600 = tof.update(600, in, true);
  require(s600.frame_fresh, "frame should remain fresh at stale threshold");
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
  auto ri = tof.update(1000, in, true);
  require(ri.reinitializing, "ACK recovery must start reinit");
  auto recovered = tof.update(1300, in, true);
  require(recovered.initialized && recovered.frame_fresh,
          "device must recover after reinit");

  in.force_reset = true;
  auto reset = tof.update(1400, in, true);
  require(reset.reinitializing && !reset.initialized,
          "reset must trigger reinitialization");
  require(tof.stats().resets == 1, "reset counter");

  in.force_reset = false;
  auto reset_done = tof.update(1700, in, true);
  require(reset_done.initialized && reset_done.frame_fresh,
          "reset recovery must deliver a frame");

  in.force_reset = true;
  tof.update(1800, in, true);
  in.force_reset = false;
  auto bus_down = tof.update(1900, in, false);
  require(!bus_down.initialized && !bus_down.reinitializing,
          "I2C loss during recovery must cancel init");
  auto bus_back = tof.update(2000, in, true);
  require(bus_back.reinitializing, "bus recovery must retry init");
  auto bus_recovered = tof.update(2300, in, true);
  require(bus_recovered.initialized && bus_recovered.frame_fresh,
          "bus recovery must restore ranging");

  std::cout << "All phase-6 VL53L5CX tests passed.\n";
  return 0;
}
