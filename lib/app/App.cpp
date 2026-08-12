#include "App.h"

namespace cores3sim {
namespace {
constexpr std::uint32_t kBootTimeMs = 500;
constexpr std::uint32_t kFaultRecoveryMs = 500;
constexpr std::uint32_t kRunTickMs = 100;
}

const char* App::stateName(RunState state) {
  switch (state) {
    case RunState::Boot: return "BOOT";
    case RunState::Ready: return "READY";
    case RunState::Running: return "RUNNING";
    case RunState::Fault: return "FAULT";
  }
  return "UNKNOWN";
}

bool App::inMainButton(int x, int y) {
  return x >= 60 && x <= 260 && y >= 175 && y <= 230;
}

UiFrame App::update(std::uint32_t now_ms,
                    const SensorSample& sensors,
                    const TouchSample& touch) {
  if (!sensors.imu_ok) {
    state_ = RunState::Fault;
    recovery_started_ms_ = 0;
  } else if (state_ == RunState::Fault) {
    if (recovery_started_ms_ == 0) {
      recovery_started_ms_ = now_ms;
    } else if (now_ms - recovery_started_ms_ >= kFaultRecoveryMs) {
      state_ = RunState::Ready;
      recovery_started_ms_ = 0;
    }
  } else if (state_ == RunState::Boot && now_ms >= kBootTimeMs) {
    state_ = RunState::Ready;
  }

  const bool touch_rising_edge = touch.pressed && !previous_touch_pressed_;
  previous_touch_pressed_ = touch.pressed;

  if (touch_rising_edge && sensors.imu_ok && inMainButton(touch.x, touch.y)) {
    if (state_ == RunState::Ready) {
      state_ = RunState::Running;
      last_run_tick_ms_ = now_ms;
    } else if (state_ == RunState::Running) {
      state_ = RunState::Ready;
    }
  }

  if (state_ == RunState::Running) {
    const std::uint32_t elapsed = now_ms - last_run_tick_ms_;
    if (elapsed >= kRunTickMs) {
      const std::uint32_t ticks = elapsed / kRunTickMs;
      run_ticks_ += ticks;
      last_run_tick_ms_ += ticks * kRunTickMs;
    }
  }

  UiFrame frame;
  frame.state = state_;
  frame.pitch_deg = sensors.pitch_deg;
  frame.battery_v = sensors.battery_v;
  frame.run_ticks = run_ticks_;

  switch (state_) {
    case RunState::Boot:
      frame.button_enabled = false;
      frame.button_label = "WAIT";
      frame.message = "Booting...";
      break;
    case RunState::Ready:
      frame.button_enabled = true;
      frame.button_label = "START";
      frame.message = "System ready";
      break;
    case RunState::Running:
      frame.button_enabled = true;
      frame.button_label = "STOP";
      frame.message = "Control loop active";
      break;
    case RunState::Fault:
      frame.button_enabled = false;
      frame.button_label = "LOCKED";
      frame.message = sensors.imu_ok ? "IMU recovering..." : "IMU unavailable";
      break;
  }

  return frame;
}

}  // namespace cores3sim
