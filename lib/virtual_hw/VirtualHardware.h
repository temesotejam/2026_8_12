#pragma once

#include <cstdint>
#include <vector>

#include "App.h"

namespace cores3sim {

struct VirtualHwInput {
  float pitch_deg{0.0f};
  float battery_v{4.0f};

  bool imu_online{true};
  bool i2c_connected{true};
  std::uint32_t i2c_latency_ms{1};

  bool uart_connected{true};
  std::uint32_t uart_latency_ms{0};
  bool gnss_source_valid{true};
};

struct VirtualHwStats {
  std::uint32_t i2c_reads{0};
  std::uint32_t i2c_timeouts{0};
  std::uint32_t uart_frames_injected{0};
  std::uint32_t uart_frames_delivered{0};
  std::uint32_t uart_frames_dropped{0};
};

class VirtualHardware {
 public:
  static constexpr std::uint32_t kI2cTimeoutMs = 10;
  static constexpr std::uint32_t kGnssStaleMs = 500;

  void apply(std::uint32_t now_ms, const VirtualHwInput& input);
  SensorSample sample(std::uint32_t now_ms);

  const VirtualHwStats& stats() const { return stats_; }

 private:
  struct PendingGnss {
    std::uint32_t deliver_at_ms{};
    std::uint32_t sequence{};
  };

  void deliverPending(std::uint32_t now_ms);

  VirtualHwInput input_{};
  VirtualHwStats stats_{};
  std::vector<PendingGnss> pending_gnss_{};

  float last_pitch_deg_{0.0f};
  bool has_gnss_{false};
  std::uint32_t last_gnss_ms_{0};
  std::uint32_t next_gnss_sequence_{1};
  bool was_uart_connected_{true};
};

}  // namespace cores3sim
