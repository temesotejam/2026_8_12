#pragma once

#include <cstdint>

#include "ProductionUartLink.h"
#include "boat_protocol.h"
#include "command_ingress.h"
#include "production_control.h"

namespace cores3sim {

enum class ProductionAction : std::uint8_t {
  None = 0,
  SetMode = 1,
  SetManual = 2,
  SetWaypoint = 3,
  SetHeading = 4,
  Arm = 5,
  Start = 6,
  Stop = 7,
  Estop = 8,
  ClearEstop = 9,
  Disarm = 10,
};

enum class PendingCommandState : std::uint8_t {
  Idle = 0,
  Waiting = 1,
  Accepted = 2,
  Rejected = 3,
  Timeout = 4,
};

struct ProductionCommandTelemetryInput {
  bool comm_to_control_connected{true};
  bool control_to_comm_connected{true};
  std::uint32_t comm_to_control_latency_ms{1};
  std::uint32_t control_to_comm_latency_ms{1};
  InjectedUartFault comm_to_control_fault{InjectedUartFault::None};
  InjectedUartFault control_to_comm_fault{InjectedUartFault::None};
  bool gnss_valid{true};

  ProductionAction action{ProductionAction::None};
  std::uint8_t mode{boat::ControlManual};
  std::uint8_t manual_mask{boat::ManualAll};
  float manual_left{0.20f};
  float manual_right{-0.20f};
  float manual_rear{0.10f};
  float manual_propulsion{0.30f};
  float heading_rad{0.0f};
  double waypoint_latitude_deg{36.40010};
  double waypoint_longitude_deg{136.60010};
  float waypoint_reach_m{1.5f};
  bool corrupt_canonical_crc{false};
};

struct ProductionCommandTelemetryStatus {
  production_control::AuthoritativeSafety safety{
      production_control::AuthoritativeSafety::Disarmed};
  production_control::StopReason stop_reason{production_control::StopReason::None};
  production_control::ControlMode control_mode{production_control::ControlMode::Manual};
  std::uint8_t manual_output_mask{0};
  bool running{false};
  bool failsafe_stop{false};

  bool comm_link_fresh{false};
  bool host_heartbeat_fresh{false};
  std::uint32_t comm_link_age_ms{0xffffffffu};
  std::uint32_t host_heartbeat_age_ms{0xffffffffu};

  PendingCommandState pending_state{PendingCommandState::Idle};
  bool pending_active{false};
  std::uint8_t pending_type{0};
  std::uint32_t pending_request_id{0};
  std::uint32_t pending_sequence{0};
  std::uint8_t pending_attempts{0};
  std::uint8_t last_control_ack_disposition{0xff};
  std::uint16_t last_control_ack_reason{0};
  std::uint8_t last_waypoint_ack_status{0xff};
  std::uint8_t last_waypoint_ack_reason{0};

  std::uint32_t control_command_acks_rx{0};
  std::uint32_t safety_command_acks_rx{0};
  std::uint32_t waypoint_acks_rx{0};
  std::uint32_t duplicate_acks_rx{0};

  std::uint32_t ingress_new{0};
  std::uint32_t ingress_applied{0};
  std::uint32_t ingress_rejected{0};
  std::uint32_t ingress_duplicates{0};
  std::uint32_t ingress_conflicts{0};
  std::uint32_t ingress_stale{0};
  std::uint32_t ingress_malformed{0};

  std::uint32_t waypoint_revision{0};
  std::uint8_t waypoint_count{0};

  std::uint32_t control_output_rx{0};
  std::uint32_t control_snapshot_rx{0};
  std::uint32_t ina_status_rx{0};
  std::uint32_t vesc_telemetry_rx{0};
  std::uint32_t actuator_state_rx{0};
  std::uint32_t system_health_rx{0};
  std::uint32_t control_heartbeat_rx{0};
  std::uint32_t gnss_nav_rx{0};
  std::uint32_t gnss_payload_crc_errors{0};

  std::uint32_t latest_snapshot_cycle{0};
  float latest_ina_bus_v{0.0f};
  float latest_ina_current_a{0.0f};
  float latest_vesc_erpm{0.0f};
  std::uint8_t latest_actuator_pca_ready{0};
  std::uint8_t latest_actuator_outputs_enabled{0};
  std::uint32_t latest_health_flags{0};
  std::uint8_t latest_health_safety{0};
  std::uint8_t latest_health_mode{0};

  std::uint8_t last_comm_rx_type{0};
  std::uint8_t last_control_rx_type{0};
};

struct ProductionCommandTelemetryStats {
  std::uint32_t pending_started{0};
  std::uint32_t pending_retries{0};
  std::uint32_t pending_accepted{0};
  std::uint32_t pending_rejected{0};
  std::uint32_t pending_timeouts{0};
  std::uint32_t pending_busy_rejections{0};
  std::uint32_t telemetry_bursts_sent{0};
  std::uint32_t gnss_sent{0};
  std::uint32_t comm_heartbeats_sent{0};
  std::uint32_t control_heartbeats_sent{0};
  std::uint32_t safety_commands_sent{0};
  std::uint32_t failsafe_stops{0};
};

struct Phase9DecoderStats {
  std::uint32_t comm_crc_errors{0};
  std::uint32_t comm_cobs_errors{0};
  std::uint32_t comm_length_errors{0};
  std::uint32_t control_crc_errors{0};
  std::uint32_t control_cobs_errors{0};
  std::uint32_t control_length_errors{0};
};

class ProductionCommandTelemetrySystem {
 public:
  static constexpr std::uint32_t kUartBaud = 921600;
  static constexpr std::uint32_t kGnssIntervalMs = 100;
  static constexpr std::uint32_t kHeartbeatIntervalMs = 100;
  static constexpr std::uint32_t kTelemetryIntervalMs = 100;
  static constexpr std::uint32_t kCommLinkFreshMs = 1000;
  static constexpr std::uint32_t kHostHeartbeatTimeoutMs = 500;
  static constexpr std::uint32_t kCommandRetryMs = 100;
  static constexpr std::uint32_t kCommandTimeoutMs = 1200;
  static constexpr std::uint8_t kCommandMaxAttempts = 8;

  ProductionCommandTelemetrySystem();

  ProductionCommandTelemetryStatus step(
      std::uint32_t now_ms,
      const ProductionCommandTelemetryInput& input);

  const ProductionCommandTelemetryStats& stats() const { return stats_; }
  const ProductionUartStats& uartStats() const { return uart_.stats(); }
  Phase9DecoderStats decoderStats() const;

 private:
  struct PendingCommand {
    boat::Type type{boat::Type::ControlModeCommand};
    std::uint8_t payload[sizeof(boat::WaypointSetPayload)]{};
    std::uint16_t length{0};
    std::uint32_t request_id{0};
    std::uint32_t sequence_or_revision{0};
    std::uint32_t started_ms{0};
    std::uint32_t last_send_ms{0};
    std::uint8_t attempts{0};
    bool active{false};
  };

  bool sendCommFrame(std::uint32_t now_ms, boat::Type type,
                     const void* payload, std::uint16_t length);
  bool sendControlFrame(std::uint32_t now_ms, boat::Type type,
                        const void* payload, std::uint16_t length);
  bool sendPending(std::uint32_t now_ms, bool retry);
  void beginPending(std::uint32_t now_ms, boat::Type type,
                    const void* payload, std::uint16_t length,
                    std::uint32_t request_id,
                    std::uint32_t sequence_or_revision);
  void servicePending(std::uint32_t now_ms);
  void handleAction(std::uint32_t now_ms,
                    const ProductionCommandTelemetryInput& input);

  void processReceives(std::uint32_t now_ms);
  void processControlRx(std::uint32_t now_ms, const boat::Frame& frame);
  void processCommRx(std::uint32_t now_ms, const boat::Frame& frame);
  void handleControlIngress(std::uint32_t now_ms, const boat::Frame& frame);
  void handleWaypointSet(std::uint32_t now_ms, const boat::Frame& frame);
  void applySafetyCommand(std::uint32_t now_ms, boat::Type type,
                          const boat::Frame& frame);
  void sendSafetyAck(std::uint32_t now_ms, boat::Type type,
                     const boat::CommandPayload& command,
                     std::uint8_t disposition,
                     std::uint16_t reason);

  void updateHealth(std::uint32_t now_ms);
  bool preflightReady(std::uint32_t now_ms,
                      const ProductionCommandTelemetryInput& input) const;
  production_control::SensorInput makeSensorInput(
      std::uint32_t now_ms,
      const ProductionCommandTelemetryInput& input) const;
  void sendGnss(std::uint32_t now_ms,
                const ProductionCommandTelemetryInput& input);
  void sendCommHeartbeat(std::uint32_t now_ms);
  void sendControlHeartbeat(std::uint32_t now_ms);
  void sendTelemetryBurst(std::uint32_t now_ms,
                          const ProductionCommandTelemetryInput& input);
  void updateIngressMetrics();

  ProductionUartLink uart_;
  boat::Decoder control_decoder_{};
  boat::Decoder comm_decoder_{};
  production_control::Controller controller_{};
  production_control::CommandIngress ingress_{};
  PendingCommand pending_{};
  ProductionCommandTelemetryStatus status_{};
  ProductionCommandTelemetryStats stats_{};

  const std::uint32_t comm_boot_id_{0x434f4d39u};
  const std::uint32_t control_boot_id_{0x43545239u};
  std::uint32_t comm_frame_sequence_{0};
  std::uint32_t control_frame_sequence_{0};
  std::uint32_t request_id_next_{1001};
  std::uint32_t command_sequence_next_{2001};
  std::uint32_t waypoint_revision_next_{1};
  std::uint32_t safety_command_id_{0};
  std::uint32_t gnss_nav_sequence_{0};
  std::uint32_t gnss_fix_sequence_{0};
  std::uint32_t telemetry_cycle_{0};
  std::uint32_t pwm_writes_{0};

  std::uint32_t last_gnss_send_ms_{0};
  std::uint32_t last_comm_hb_send_ms_{0};
  std::uint32_t last_control_hb_send_ms_{0};
  std::uint32_t last_telemetry_send_ms_{0};
  std::uint32_t last_control_frame_rx_ms_{0};
  std::uint32_t last_host_hb_rx_ms_{0};
  bool has_control_frame_{false};
  bool has_host_hb_{false};

  std::uint32_t waypoint_revision_{0};
  std::uint8_t waypoint_count_{0};
  boat::WaypointGeo waypoint_geo_[16]{};
  float waypoint_reach_m_{1.5f};
};

}  // namespace cores3sim
