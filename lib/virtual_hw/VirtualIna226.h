#pragma once

#include <cstdint>

namespace cores3sim {

struct Ina226Input {
  bool enabled{false};
  bool powered{true};
  bool device_ack{true};
  bool force_reset{false};
  bool conversion_enabled{true};
  bool stall_conversions{false};
  bool calibration_programmed{true};
  bool force_math_overflow{false};

  float bus_voltage_v{12.0f};
  float current_a{0.0f};
  float shunt_ohm{0.002f};
  float current_lsb_a{0.00125f};

  std::uint32_t shunt_conversion_us{1100};
  std::uint32_t bus_conversion_us{1100};
  std::uint32_t averages{1};
  std::uint32_t reinit_delay_ms{20};
  bool auto_reinit{true};
};

struct Ina226Status {
  bool active{false};
  bool device_ok{true};
  bool initialized{true};
  bool reinitializing{false};
  bool configuration_ok{true};
  bool conversion_fresh{true};
  bool calibration_ok{true};
  bool range_ok{true};
  bool math_overflow{false};
  std::uint32_t measurement_age_ms{0};

  float bus_voltage_v{0.0f};
  float current_a{0.0f};
  float power_w{0.0f};
  float shunt_voltage_v{0.0f};
};

struct Ina226Stats {
  std::uint32_t resets{0};
  std::uint32_t reinit_attempts{0};
  std::uint32_t reinit_successes{0};
  std::uint32_t conversions_completed{0};
  std::uint32_t stale_events{0};
  std::uint32_t device_nacks{0};
  std::uint32_t config_errors{0};
  std::uint32_t range_errors{0};
  std::uint32_t math_overflows{0};
  std::uint32_t calibration_missing_conversions{0};
};

class VirtualIna226 {
 public:
  static constexpr float kMinShuntVoltageV = -0.0819175f;
  static constexpr float kMaxShuntVoltageV = 0.08192f;
  static constexpr float kMaxBusVoltageV = 36.0f;

  Ina226Status update(std::uint32_t now_ms,
                      const Ina226Input& input,
                      bool bus_ok);

  const Ina226Stats& stats() const { return stats_; }

 private:
  static bool validConversionTime(std::uint32_t us);
  static bool validAverages(std::uint32_t averages);
  static std::uint32_t conversionCycleMs(const Ina226Input& input);
  static float quantize(float value, float lsb);

  void clearRuntime();
  void startReinit(std::uint32_t now_ms, std::uint32_t delay_ms);

  Ina226Stats stats_{};
  bool initialized_{false};
  bool reinitializing_{false};
  bool has_measurement_{false};
  bool reset_latched_{false};
  bool stale_latched_{false};
  bool ack_missing_latched_{false};

  std::uint32_t reinit_ready_at_ms_{0};
  std::uint32_t next_conversion_ready_ms_{0};
  std::uint32_t last_measurement_ms_{0};

  bool last_range_ok_{true};
  bool last_math_overflow_{false};
  float last_bus_voltage_v_{0.0f};
  float last_current_a_{0.0f};
  float last_power_w_{0.0f};
  float last_shunt_voltage_v_{0.0f};
};

}  // namespace cores3sim
