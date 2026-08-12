#pragma once

#include <cstdint>
#include <vector>

#include "App.h"
#include "VirtualBno08x.h"
#include "VirtualIna226.h"

namespace cores3sim {

struct VirtualHwInput {
  float pitch_deg{0.0f};
  float battery_v{4.0f};

  bool imu_online{true};
  bool i2c_connected{true};
  std::uint32_t i2c_latency_ms{1};
  bool i2c_nack{false};

  bool uart_connected{true};
  std::uint32_t uart_latency_ms{0};
  bool gnss_source_valid{true};
  bool uart_corrupt_byte{false};
  bool uart_crc_error{false};
  bool uart_framing_error{false};

  std::int32_t loop_jitter_ms{0};

  bool sd_connected{true};
  bool sd_fail_write{false};
  std::uint32_t sd_latency_ms{1};

  // Phase 4: optional device-specific BNO08X model.
  // When false, phase 1-3 direct IMU behavior is preserved.
  bool bno_model_enabled{false};
  bool bno_force_reset{false};
  bool bno_reports_enabled{true};
  bool bno_stall_reports{false};
  std::uint32_t bno_report_interval_ms{20};
  std::uint32_t bno_reinit_delay_ms{300};
  bool bno_auto_reinit{true};

  // Phase 5: optional device-specific INA226 model.
  bool ina_model_enabled{false};
  bool ina_powered{true};
  bool ina_device_ack{true};
  bool ina_force_reset{false};
  bool ina_conversion_enabled{true};
  bool ina_stall_conversions{false};
  bool ina_calibration_programmed{true};
  bool ina_force_math_overflow{false};
  float ina_bus_voltage_v{12.0f};
  float ina_current_a{0.0f};
  float ina_shunt_ohm{0.002f};
  float ina_current_lsb_a{0.00125f};
  std::uint32_t ina_shunt_conversion_us{1100};
  std::uint32_t ina_bus_conversion_us{1100};
  std::uint32_t ina_averages{1};
  std::uint32_t ina_reinit_delay_ms{20};
  bool ina_auto_reinit{true};
};

struct VirtualHwStats {
  std::uint32_t i2c_reads{0};
  std::uint32_t i2c_timeouts{0};
  std::uint32_t i2c_nacks{0};
  std::uint32_t uart_frames_injected{0};
  std::uint32_t uart_frames_delivered{0};
  std::uint32_t uart_frames_dropped{0};
  std::uint32_t uart_corrupt_bytes{0};
  std::uint32_t uart_crc_errors{0};
  std::uint32_t uart_framing_errors{0};
  std::uint32_t timing_jitter_events{0};
  std::uint32_t max_abs_jitter_ms{0};
  std::uint32_t sd_write_attempts{0};
  std::uint32_t sd_write_failures{0};
  std::uint32_t sd_write_timeouts{0};
};

class VirtualHardware {
 public:
  static constexpr std::uint32_t kI2cTimeoutMs = 10;
  static constexpr std::uint32_t kGnssStaleMs = 500;
  static constexpr std::uint32_t kLoopJitterWarnMs = 20;
  static constexpr std::uint32_t kSdTimeoutMs = 20;

  void apply(std::uint32_t now_ms, const VirtualHwInput& input);
  SensorSample sample(std::uint32_t now_ms);

  const VirtualHwStats& stats() const { return stats_; }
  const Bno08xStats& bnoStats() const { return bno_.stats(); }
  const Ina226Stats& inaStats() const { return ina_.stats(); }

 private:
  struct PendingGnss {
    std::uint32_t deliver_at_ms{};
    std::uint32_t sequence{};
  };

  void deliverPending(std::uint32_t now_ms);

  VirtualHwInput input_{};
  VirtualHwStats stats_{};
  std::vector<PendingGnss> pending_gnss_{};
  VirtualBno08x bno_{};
  VirtualIna226 ina_{};

  float last_pitch_deg_{0.0f};
  bool has_gnss_{false};
  std::uint32_t last_gnss_ms_{0};
  std::uint32_t next_gnss_sequence_{1};
  bool was_uart_connected_{true};
  bool uart_frame_ok_{true};
};

}  // namespace cores3sim
