#pragma once

#include <cstdint>
#include <string>

namespace cores3sim {

enum class RunState {
  Boot,
  Ready,
  Running,
  Fault,
};

struct SensorSample {
  // Keep the first three fields compatible with the phase-1 aggregate initializer.
  bool imu_ok{true};
  float pitch_deg{0.0f};
  float battery_v{4.0f};

  bool i2c_ok{true};
  bool uart_ok{true};
  bool gnss_ok{true};
  std::uint32_t gnss_age_ms{0};

  // Phase 3 transport/logging/timing health. Appended for source compatibility.
  bool i2c_nack{false};
  bool uart_frame_ok{true};
  bool timing_ok{true};
  std::int32_t loop_jitter_ms{0};
  bool sd_ok{true};

  // Phase 4 BNO08X model health. Appended for source compatibility.
  bool bno_model_active{false};
  bool bno_initialized{true};
  bool bno_report_fresh{true};
  bool bno_reinitializing{false};
  std::uint32_t bno_report_age_ms{0};

  // Phase 5 INA226 model health and converted output registers.
  bool ina_model_active{false};
  bool ina_device_ok{true};
  bool ina_initialized{true};
  bool ina_reinitializing{false};
  bool ina_conversion_fresh{true};
  bool ina_calibration_ok{true};
  bool ina_range_ok{true};
  bool ina_math_overflow{false};
  std::uint32_t ina_measurement_age_ms{0};
  float ina_bus_voltage_v{0.0f};
  float ina_current_a{0.0f};
  float ina_power_w{0.0f};
  float ina_shunt_voltage_v{0.0f};

  // Phase 6 VL53L5CX 8x8 frame health.
  bool tof_model_active{false};
  bool tof_device_ok{true};
  bool tof_initialized{true};
  bool tof_reinitializing{false};
  bool tof_frame_fresh{true};
  std::uint32_t tof_frame_age_ms{0};
  std::uint8_t tof_valid_zones{64};
  std::uint16_t tof_min_distance_mm{0};
};

struct TouchSample {
  bool pressed{false};
  int x{0};
  int y{0};
};

struct UiFrame {
  RunState state{RunState::Boot};
  float pitch_deg{0.0f};
  float battery_v{0.0f};
  std::uint32_t run_ticks{0};
  bool button_enabled{false};
  bool i2c_ok{true};
  bool imu_ok{true};
  bool uart_ok{true};
  bool gnss_ok{true};
  std::uint32_t gnss_age_ms{0};
  bool i2c_nack{false};
  bool uart_frame_ok{true};
  bool timing_ok{true};
  std::int32_t loop_jitter_ms{0};
  bool sd_ok{true};
  bool bno_model_active{false};
  bool bno_initialized{true};
  bool bno_report_fresh{true};
  bool bno_reinitializing{false};
  std::uint32_t bno_report_age_ms{0};
  bool ina_model_active{false};
  bool ina_device_ok{true};
  bool ina_initialized{true};
  bool ina_reinitializing{false};
  bool ina_conversion_fresh{true};
  bool ina_calibration_ok{true};
  bool ina_range_ok{true};
  bool ina_math_overflow{false};
  std::uint32_t ina_measurement_age_ms{0};
  float ina_bus_voltage_v{0.0f};
  float ina_current_a{0.0f};
  float ina_power_w{0.0f};
  float ina_shunt_voltage_v{0.0f};
  bool tof_model_active{false};
  bool tof_device_ok{true};
  bool tof_initialized{true};
  bool tof_reinitializing{false};
  bool tof_frame_fresh{true};
  std::uint32_t tof_frame_age_ms{0};
  std::uint8_t tof_valid_zones{64};
  std::uint16_t tof_min_distance_mm{0};
  std::string button_label{"WAIT"};
  std::string message{"Booting"};
  std::string warning{};
};

class App {
 public:
  App() = default;

  UiFrame update(std::uint32_t now_ms,
                 const SensorSample& sensors,
                 const TouchSample& touch);

  RunState state() const { return state_; }
  std::uint32_t runTicks() const { return run_ticks_; }

  static const char* stateName(RunState state);
  static bool inMainButton(int x, int y);

 private:
  RunState state_{RunState::Boot};
  bool previous_touch_pressed_{false};
  std::uint32_t recovery_started_ms_{0};
  std::uint32_t last_run_tick_ms_{0};
  std::uint32_t run_ticks_{0};
};

}  // namespace cores3sim
