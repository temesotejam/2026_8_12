#pragma once

#include <cstdint>
#include <vector>

#include "ProductionUartLink.h"
#include "boat_protocol.h"

namespace cores3sim {

enum class ProductionSafety : std::uint8_t {
  Boot = 0,
  Disarmed = 1,
  Armed = 2,
  Running = 3,
  EStop = 4,
  Fault = 5,
};

enum class ProductionStopReason : std::uint8_t {
  None = 0,
  Stop = 1,
  EStop = 2,
  Heartbeat = 3,
};

struct ProductionProtocolInput {
  bool comm_to_control_connected{true};
  bool control_to_comm_connected{true};
  std::uint32_t comm_to_control_latency_ms{1};
  std::uint32_t control_to_comm_latency_ms{1};
  InjectedUartFault comm_to_control_fault{InjectedUartFault::None};
  InjectedUartFault control_to_comm_fault{InjectedUartFault::None};
  bool gnss_valid{true};
  bool send_arm{false};
  bool send_start{false};
  bool send_stop{false};
  bool send_estop{false};
  bool send_clear_estop{false};
  bool send_disarm{false};
};

struct ProductionProtocolStatus {
  ProductionSafety control_safety{ProductionSafety::Disarmed};
  ProductionStopReason stop_reason{ProductionStopReason::None};
  bool control_running{false};
  bool control_failsafe_stop{false};
  bool comm_link_fresh{false};
  bool control_host_heartbeat_fresh{false};
  std::uint32_t comm_link_age_ms{0xffffffffu};
  std::uint32_t host_heartbeat_age_ms{0xffffffffu};
  std::uint32_t gnss_received{0};
  std::uint32_t control_frames_received{0};
  std::uint32_t command_acks_received{0};
  std::uint32_t stop_received{0};
  std::uint32_t estop_received{0};
  std::uint32_t clear_estop_received{0};
  std::uint32_t arm_received{0};
  std::uint32_t start_received{0};
  std::uint32_t disarm_received{0};
  std::uint32_t gnss_canonical_crc_errors{0};
  std::uint8_t last_comm_rx_type{0};
  std::uint8_t last_control_rx_type{0};
  std::uint32_t last_comm_rx_sequence{0};
  std::uint32_t last_control_rx_sequence{0};
  std::uint8_t last_ack_command_type{0};
  std::uint8_t last_ack_disposition{0};
};

struct ProductionProtocolStats {
  std::uint32_t comm_frames_sent{0};
  std::uint32_t control_frames_sent{0};
  std::uint32_t gnss_sent{0};
  std::uint32_t comm_heartbeats_sent{0};
  std::uint32_t control_heartbeats_sent{0};
  std::uint32_t control_outputs_sent{0};
  std::uint32_t safety_commands_sent{0};
  std::uint32_t command_acks_sent{0};
  std::uint32_t failsafe_stops{0};
};

struct ProtocolDecoderStats {
  std::uint32_t comm_crc_errors{0};
  std::uint32_t comm_cobs_errors{0};
  std::uint32_t comm_length_errors{0};
  std::uint32_t control_crc_errors{0};
  std::uint32_t control_cobs_errors{0};
  std::uint32_t control_length_errors{0};
};

class ProductionProtocolSystem {
 public:
  static constexpr std::uint32_t kUartBaud = 921600;
  static constexpr std::uint32_t kGnssIntervalMs = 100;
  static constexpr std::uint32_t kHeartbeatIntervalMs = 100;
  static constexpr std::uint32_t kControlTelemetryIntervalMs = 100;
  static constexpr std::uint32_t kCommLinkFreshMs = 1000;
  static constexpr std::uint32_t kHostHeartbeatTimeoutMs = 500;

  ProductionProtocolSystem();

  ProductionProtocolStatus step(std::uint32_t now_ms,
                                const ProductionProtocolInput& input);

  const ProductionProtocolStats& stats() const { return stats_; }
  const ProductionUartStats& uartStats() const { return uart_.stats(); }
  ProtocolDecoderStats decoderStats() const;

 private:
  bool sendCommFrame(std::uint32_t now_ms, boat::Type type,
                     const void* payload, std::uint16_t length);
  bool sendControlFrame(std::uint32_t now_ms, boat::Type type,
                        const void* payload, std::uint16_t length);
  void sendDue(std::uint32_t now_ms, const ProductionProtocolInput& input);
  void processReceives(std::uint32_t now_ms);
  void processControlFrame(std::uint32_t now_ms, const boat::Frame& frame);
  void processCommFrame(std::uint32_t now_ms, const boat::Frame& frame);
  void sendSafetyCommand(std::uint32_t now_ms, boat::Type type);
  void sendSafetyAck(std::uint32_t now_ms, const boat::CommandPayload& command,
                     boat::Type type, std::uint8_t disposition,
                     std::uint16_t reason = 0);
  void applySafetyCommand(std::uint32_t now_ms, boat::Type type,
                          const boat::CommandPayload& command);
  void updateHealth(std::uint32_t now_ms);

  ProductionUartLink uart_;
  boat::Decoder control_decoder_{};
  boat::Decoder comm_decoder_{};
  ProductionProtocolStatus status_{};
  ProductionProtocolStats stats_{};

  const std::uint32_t comm_boot_id_{0x434f4d4du};
  const std::uint32_t control_boot_id_{0x4354524cu};
  std::uint32_t comm_frame_sequence_{0};
  std::uint32_t control_frame_sequence_{0};
  std::uint32_t gnss_nav_sequence_{0};
  std::uint32_t gnss_fix_sequence_{0};
  std::uint32_t safety_command_id_{0};
  std::uint32_t last_gnss_send_ms_{0};
  std::uint32_t last_comm_hb_send_ms_{0};
  std::uint32_t last_control_hb_send_ms_{0};
  std::uint32_t last_control_telemetry_send_ms_{0};
  std::uint32_t last_control_frame_rx_ms_{0};
  std::uint32_t last_host_hb_rx_ms_{0};
  bool has_control_frame_{false};
  bool has_host_hb_{false};
  std::uint8_t comm_cached_safety_{static_cast<std::uint8_t>(ProductionSafety::Disarmed)};
};

}  // namespace cores3sim
