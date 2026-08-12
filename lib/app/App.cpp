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
  const bool startup_bno_wait =
      state_ == RunState::Boot &&
      sensors.bno_model_active &&
      sensors.bno_reinitializing;

  if (!critical_ok && !startup_bno_wait) {
    state_ = RunState::Fault;
    recovery_started_ms_ = 0;
  } else if (state_ == RunState::Fault) {
    if (critical_ok) {
      if (recovery_started_ms_ == 0) {
        recovery_started_ms_ = now_ms;
      } else if (now_ms - recovery_started_ms_ >= kFaultRecoveryMs) {
        state_ = RunState::Ready;
        recovery_started_ms_ = 0;
      }
    }
  } else if (state_ == RunState::Boot &&
             now_ms >= kBootTimeMs &&
             critical_ok) {
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
  frame.bno_model_active = sensors.bno_model_active;
  frame.bno_initialized = sensors.bno_initialized;
  frame.bno_report_fresh = sensors.bno_report_fresh;
  frame.bno_reinitializing = sensors.bno_reinitializing;
  frame.bno_report_age_ms = sensors.bno_report_age_ms;
  frame.ina_model_active = sensors.ina_model_active;
  frame.ina_device_ok = sensors.ina_device_ok;
  frame.ina_initialized = sensors.ina_initialized;
  frame.ina_reinitializing = sensors.ina_reinitializing;
  frame.ina_conversion_fresh = sensors.ina_conversion_fresh;
  frame.ina_calibration_ok = sensors.ina_calibration_ok;
  frame.ina_range_ok = sensors.ina_range_ok;
  frame.ina_math_overflow = sensors.ina_math_overflow;
  frame.ina_measurement_age_ms = sensors.ina_measurement_age_ms;
  frame.ina_bus_voltage_v = sensors.ina_bus_voltage_v;
  frame.ina_current_a = sensors.ina_current_a;
  frame.ina_power_w = sensors.ina_power_w;
  frame.ina_shunt_voltage_v = sensors.ina_shunt_voltage_v;
  frame.tof_model_active = sensors.tof_model_active;
  frame.tof_device_ok = sensors.tof_device_ok;
  frame.tof_initialized = sensors.tof_initialized;
  frame.tof_reinitializing = sensors.tof_reinitializing;
  frame.tof_frame_fresh = sensors.tof_frame_fresh;
  frame.tof_frame_age_ms = sensors.tof_frame_age_ms;
  frame.tof_valid_zones = sensors.tof_valid_zones;
  frame.tof_min_distance_mm = sensors.tof_min_distance_mm;

  if (!sensors.uart_ok) {
    addWarning(frame.warning, "UART link down");
  } else if (!sensors.uart_frame_ok) {
    addWarning(frame.warning, "UART frame error");
  }
  if (!sensors.gnss_ok) addWarning(frame.warning, "GNSS data stale");
  if (!sensors.timing_ok) addWarning(frame.warning, "Control loop jitter");
  if (!sensors.sd_ok) addWarning(frame.warning, "SD write failed");

  if (sensors.ina_model_active) {
    if (!sensors.ina_device_ok) {
      addWarning(frame.warning, "INA226 offline");
    } else if (sensors.ina_reinitializing) {
      addWarning(frame.warning, "INA226 init");
    } else if (!sensors.ina_initialized) {
      addWarning(frame.warning, "INA226 uninitialized");
    } else if (!sensors.ina_conversion_fresh) {
      addWarning(frame.warning, "INA226 data stale");
    }
    if (!sensors.ina_calibration_ok) {
      addWarning(frame.warning, "INA226 uncalibrated");
    }
    if (!sensors.ina_range_ok) {
      addWarning(frame.warning, "INA226 input range");
    }
    if (sensors.ina_math_overflow) {
      addWarning(frame.warning, "INA226 math overflow");
    }
  }

  if (sensors.tof_model_active) {
    if (!sensors.tof_device_ok) {
      addWarning(frame.warning, "VL53L5CX offline");
    } else if (sensors.tof_reinitializing) {
      addWarning(frame.warning, "VL53L5CX init");
    } else if (!sensors.tof_initialized) {
      addWarning(frame.warning, "VL53L5CX uninitialized");
    } else if (!sensors.tof_frame_fresh) {
      addWarning(frame.warning, "VL53L5CX frame stale");
    }
    if (sensors.tof_initialized && sensors.tof_valid_zones < 64) {
      addWarning(frame.warning, "VL53L5CX invalid zones");
    }
  }

  switch (state_) {
    case RunState::Boot:
      frame.button_enabled = false;
      frame.button_label = "WAIT";
      frame.message = sensors.bno_model_active && sensors.bno_reinitializing
                          ? "BNO08X startup init"
                          : "Booting...";
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
      } else if (sensors.bno_model_active && sensors.bno_reinitializing) {
        frame.message = "BNO08X reinitializing";
      } else if (sensors.bno_model_active && !sensors.bno_initialized) {
        frame.message = "BNO08X offline";
      } else if (sensors.bno_model_active && !sensors.bno_report_fresh) {
        frame.message = "BNO08X report stale";
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
