#include "App.h"

#include <cstdlib>
#include <iostream>

using cores3sim::App;
using cores3sim::RunState;
using cores3sim::SensorSample;
using cores3sim::TouchSample;

namespace {
void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}
}

int main() {
  App app;
  SensorSample sensor{true, 0.0f, 4.05f};
  TouchSample none{};

  require(app.update(0, sensor, none).state == RunState::Boot,
          "state must start at BOOT");
  require(app.update(499, sensor, none).state == RunState::Boot,
          "BOOT must last 500 ms");
  require(app.update(500, sensor, none).state == RunState::Ready,
          "state must become READY after boot");

  TouchSample press{true, 160, 205};
  require(app.update(600, sensor, press).state == RunState::Running,
          "button press must start RUNNING");
  require(app.update(750, sensor, press).state == RunState::Running,
          "holding the button must not toggle repeatedly");
  require(app.runTicks() == 1,
          "RUNNING counter must advance every 100 ms");

  require(app.update(800, sensor, none).state == RunState::Running,
          "release must keep RUNNING");
  require(app.update(900, sensor, press).state == RunState::Ready,
          "second press must stop and return READY");
  app.update(950, sensor, none);

  sensor.imu_ok = false;
  require(app.update(1000, sensor, none).state == RunState::Fault,
          "IMU loss must force FAULT");
  require(!app.update(1100, sensor, press).button_enabled,
          "button must be disabled in FAULT");

  app.update(1200, sensor, none);
  sensor.imu_ok = true;
  require(app.update(1300, sensor, none).state == RunState::Fault,
          "FAULT must not clear immediately");
  require(app.update(1799, sensor, none).state == RunState::Fault,
          "recovery must be stable for 500 ms");
  require(app.update(1800, sensor, none).state == RunState::Ready,
          "stable IMU recovery must return READY");

  require(App::inMainButton(160, 205), "button center must be inside");
  require(!App::inMainButton(10, 10), "screen corner must be outside button");

  std::cout << "All app logic tests passed.\n";
  return 0;
}
