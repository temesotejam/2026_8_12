#pragma once

#include <cstdint>

#include "VirtualDuplexUart.h"

namespace cores3sim {

struct DualControllerInput {
  bool comm_to_control_connected{true};
  bool control_to_comm_connected{true};
  std::uint32_t comm_to_control_latency_ms{1};
  std::uint32_t control_to_comm_latency_ms{1};
  InjectedUartFault comm_to_control_fault{InjectedUartFault::None};
  InjectedUartFault control_to_comm_fault{InjectedUartFault::None};
  bool gnss_valid{true};
  bool send_stop{false};
};

struct DualControllerStatus {
  bool control_running{true};
  bool control_failsafe_stop{false};
  bool comm_sees_control_link{false};
  bool control_sees_comm_link{false};
  std::uint32_t comm_heartbeat_age_ms{0};
  std::uint32_t control_heartbeat_age_ms{0};
  std::uint32_t gnss_received{0};
  std::uint32_t results_received{0};
  std::uint32_t stop_received{0};
};

struct DualControllerStats {
  std::uint32_t gnss_sent{0};
  std::uint32_t results_sent{0};
  std::uint32_t comm_heartbeats_sent{0};
  std::uint32_t control_heartbeats_sent{0};
  std::uint32_t stop_sent{0};
  std::uint32_t failsafe_stops{0};
};

class DualControllerSystem {
 public:
  static constexpr std::uint32_t kUartBaud = 921600;
  static constexpr std::uint32_t kHeartbeatIntervalMs = 100;
  static constexpr std::uint32_t kDataIntervalMs = 100;
  static constexpr std::uint32_t kHeartbeatTimeoutMs = 500;

  DualControllerSystem();

  DualControllerStatus step(std::uint32_t now_ms,
                            const DualControllerInput& input);

  const DualControllerStats& stats() const { return stats_; }
  const DuplexUartStats& uartStats() const { return uart_.stats(); }

 private:
  void processReceives(std::uint32_t now_ms);
  void updateHealth(std::uint32_t now_ms);
  void sendDue(std::uint32_t now_ms, const DualControllerInput& input);

  VirtualDuplexUart uart_;
  DualControllerStats stats_{};
  DualControllerStatus status_{};

  std::uint32_t last_comm_hb_send_ms_{0};
  std::uint32_t last_control_hb_send_ms_{0};
  std::uint32_t last_gnss_send_ms_{0};
  std::uint32_t last_result_send_ms_{0};
  std::uint32_t last_comm_hb_rx_ms_{0};
  std::uint32_t last_control_hb_rx_ms_{0};
  bool has_comm_hb_{false};
  bool has_control_hb_{false};

  std::uint32_t next_gnss_seq_{1};
  std::uint32_t next_result_seq_{1};
  std::uint32_t next_comm_hb_seq_{1};
  std::uint32_t next_control_hb_seq_{1};
  std::uint32_t next_stop_seq_{1};
};

}  // namespace cores3sim
