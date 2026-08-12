#include "App.h"

namespace cores3sim {
namespace {
constexpr std::uint32_t kBootTimeMs = 500;
constexpr std::uint32_t kFaultRecoveryMs = 500;
constexpr std::uint32_t kRunTickMs = 100;

void addWarning(std::string& dst, const std::string& text) {
  if (!dst.empty()) dst += " | ";
  dst += text;
}
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
  const bool critical_ok = sensors.i2c_ok && sensors.imu_ok;

  if (!critical_ok) {
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

  if (touch_rising_edge && critical_ok && inMainButton(touch.x, touch.y)) {
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
  frame.i2c_ok = sensors.i2c_ok;
  frame.imu_ok = sensors.imu_ok;
  frame.uart_ok = sensors.uart_ok;
  frame.gnss_ok = sensors.gnss_ok;
  frame.gnss_age_ms = sensors.gnss_age_ms;
  frame.i2c_nack = sensors.i2c_nack;
  frame.uart_frame_ok = sensors.uart_frame_ok;
  frame.timing_ok = sensors.timing_ok;
  frame.loop_jitter_ms = sensors.loop_jitter_ms;
  frame.sd_ok = sensors.sd_ok;

  if (!sensors.uart_ok) {
    addWarning(frame.warning, "UART link down");
  } else if (!sensors.uart_frame_ok) {
    addWarning(frame.warning, "UART frame error");
  }
  if (!sensors.gnss_ok) addWarning(frame.warning, "GNSS data stale");
  if (!sensors.timing_ok) addWarning(frame.warning, "Control loop jitter");
  if (!sensors.sd_ok) addWarning(frame.warning, "SD write failed");

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
      if (sensors.i2c_nack) {
        frame.message = "I2C device NACK";
      } else if (!sensors.i2c_ok) {
        frame.message = "I2C timeout / disconnect";
      } else if (!sensors.imu_ok) {
        frame.message = "IMU unavailable";
      } else {
        frame.message = "Critical sensor recovering...";
      }
      break;
  }

  return frame;
}

}  // namespace cores3sim
