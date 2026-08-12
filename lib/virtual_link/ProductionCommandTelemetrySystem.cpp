#include "ProductionCommandTelemetrySystem.h"

#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>

namespace cores3sim {
namespace {
constexpr double kOriginLatDeg = 36.40000;
constexpr double kOriginLonDeg = 136.60000;
constexpr std::uint32_t kAllGnssFlags =
    boat::NavFixValid | boat::NavNewFix | boat::NavLatValid |
    boat::NavLonValid | boat::NavAltitudeValid | boat::NavSpeedValid |
    boat::NavCourseValid | boat::NavHdopValid;

std::uint32_t ageOrMax(bool valid, std::uint32_t now, std::uint32_t stamp) {
  return valid ? now - stamp : std::numeric_limits<std::uint32_t>::max();
}

std::uint16_t servoPulse(float value) {
  if (!std::isfinite(value)) return 0;
  if (value < -1.0f) value = -1.0f;
  if (value > 1.0f) value = 1.0f;
  return static_cast<std::uint16_t>(1500.0f + value * 300.0f);
}
}

ProductionCommandTelemetrySystem::ProductionCommandTelemetrySystem()
    : uart_(kUartBaud, 32768) {
  status_.safety = production_control::AuthoritativeSafety::Disarmed;
  status_.control_mode = controller_.mode();
}

bool ProductionCommandTelemetrySystem::sendCommFrame(
    std::uint32_t now_ms, boat::Type type,
    const void* payload, std::uint16_t length) {
  if (length > boat::kMaxPayload) return false;
  boat::Header header{boat::kVersion,
                      static_cast<std::uint8_t>(type),
                      length,
                      ++comm_frame_sequence_,
                      comm_boot_id_,
                      static_cast<std::uint64_t>(now_ms) * 1000ULL,
                      0};
  std::uint8_t encoded[boat::kMaxEncoded]{};
  const std::size_t bytes = boat::encode(
      header, static_cast<const std::uint8_t*>(payload), encoded, sizeof(encoded));
  return bytes && uart_.send(UartDirection::CommToControl, now_ms, encoded, bytes);
}

bool ProductionCommandTelemetrySystem::sendControlFrame(
    std::uint32_t now_ms, boat::Type type,
    const void* payload, std::uint16_t length) {
  if (length > boat::kMaxPayload) return false;
  boat::Header header{boat::kVersion,
                      static_cast<std::uint8_t>(type),
                      length,
                      ++control_frame_sequence_,
                      control_boot_id_,
                      static_cast<std::uint64_t>(now_ms) * 1000ULL,
                      0};
  std::uint8_t encoded[boat::kMaxEncoded]{};
  const std::size_t bytes = boat::encode(
      header, static_cast<const std::uint8_t*>(payload), encoded, sizeof(encoded));
  return bytes && uart_.send(UartDirection::ControlToComm, now_ms, encoded, bytes);
}

bool ProductionCommandTelemetrySystem::sendPending(std::uint32_t now_ms,
                                                   bool retry) {
  if (!pending_.active) return false;
  const bool sent = sendCommFrame(now_ms, pending_.type,
                                  pending_.payload, pending_.length);
  if (sent) {
    pending_.last_send_ms = now_ms;
    ++pending_.attempts;
    status_.pending_attempts = pending_.attempts;
    if (retry) ++stats_.pending_retries;
  }
  return sent;
}

void ProductionCommandTelemetrySystem::beginPending(
    std::uint32_t now_ms, boat::Type type,
    const void* payload, std::uint16_t length,
    std::uint32_t request_id,
    std::uint32_t sequence_or_revision) {
  if (pending_.active) {
    ++stats_.pending_busy_rejections;
    return;
  }
  pending_ = {};
  pending_.type = type;
  pending_.length = length;
  pending_.request_id = request_id;
  pending_.sequence_or_revision = sequence_or_revision;
  pending_.started_ms = now_ms;
  pending_.active = true;
  std::memcpy(pending_.payload, payload, length);

  status_.pending_state = PendingCommandState::Waiting;
  status_.pending_active = true;
  status_.pending_type = static_cast<std::uint8_t>(type);
  status_.pending_request_id = request_id;
  status_.pending_sequence = sequence_or_revision;
  status_.pending_attempts = 0;
  status_.last_control_ack_disposition = 0xff;
  status_.last_control_ack_reason = 0;
  status_.last_waypoint_ack_status = 0xff;
  status_.last_waypoint_ack_reason = 0;
  ++stats_.pending_started;
  sendPending(now_ms, false);
}

void ProductionCommandTelemetrySystem::servicePending(std::uint32_t now_ms) {
  if (!pending_.active) return;
  if (now_ms - pending_.started_ms > kCommandTimeoutMs) {
    pending_.active = false;
    status_.pending_active = false;
    status_.pending_state = PendingCommandState::Timeout;
    ++stats_.pending_timeouts;
    return;
  }
  if ((!pending_.last_send_ms || now_ms - pending_.last_send_ms >= kCommandRetryMs) &&
      pending_.attempts < kCommandMaxAttempts) {
    sendPending(now_ms, pending_.attempts > 0);
  }
}

void ProductionCommandTelemetrySystem::handleAction(
    std::uint32_t now_ms,
    const ProductionCommandTelemetryInput& input) {
  if (input.action == ProductionAction::None) return;

  if (input.action == ProductionAction::SetMode) {
    boat::ControlModeCommandPayload command{};
    command.protocolVersion = boat::kVersion;
    command.mode = input.mode;
    command.requestId = request_id_next_++;
    command.commandSequence = command_sequence_next_++;
    command.sourceUs = static_cast<std::uint64_t>(now_ms) * 1000ULL;
    command.canonicalCrc = boat::canonicalCrc(
        &command, offsetof(boat::ControlModeCommandPayload, canonicalCrc));
    if (input.corrupt_canonical_crc) command.canonicalCrc ^= 1u;
    beginPending(now_ms, boat::Type::ControlModeCommand,
                 &command, sizeof(command),
                 command.requestId, command.commandSequence);
    return;
  }

  if (input.action == ProductionAction::SetManual) {
    boat::ManualCommandPayload command{};
    command.protocolVersion = boat::kVersion;
    command.reserved[0] = input.manual_mask;
    command.requestId = request_id_next_++;
    command.commandSequence = command_sequence_next_++;
    command.sourceUs = static_cast<std::uint64_t>(now_ms) * 1000ULL;
    command.leftFrontWing = input.manual_left;
    command.rightFrontWing = input.manual_right;
    command.rearYaw = input.manual_rear;
    command.propulsion = input.manual_propulsion;
    command.canonicalCrc = boat::canonicalCrc(
        &command, offsetof(boat::ManualCommandPayload, canonicalCrc));
    if (input.corrupt_canonical_crc) command.canonicalCrc ^= 1u;
    beginPending(now_ms, boat::Type::ManualCommand,
                 &command, sizeof(command),
                 command.requestId, command.commandSequence);
    return;
  }

  if (input.action == ProductionAction::SetHeading) {
    boat::HeadingTargetPayload command{};
    command.protocolVersion = boat::kVersion;
    command.requestId = request_id_next_++;
    command.commandSequence = command_sequence_next_++;
    command.sourceUs = static_cast<std::uint64_t>(now_ms) * 1000ULL;
    command.targetYawRad = input.heading_rad;
    command.canonicalCrc = boat::canonicalCrc(
        &command, offsetof(boat::HeadingTargetPayload, canonicalCrc));
    if (input.corrupt_canonical_crc) command.canonicalCrc ^= 1u;
    beginPending(now_ms, boat::Type::HeadingTarget,
                 &command, sizeof(command),
                 command.requestId, command.commandSequence);
    return;
  }

  if (input.action == ProductionAction::SetWaypoint) {
    boat::WaypointSetPayload command{};
    command.requestId = request_id_next_++;
    command.revision = waypoint_revision_next_++;
    command.action = 1;
    command.count = 1;
    command.reachRadiusM = input.waypoint_reach_m;
    command.points[0].latitudeDeg = input.waypoint_latitude_deg;
    command.points[0].longitudeDeg = input.waypoint_longitude_deg;
    command.canonicalCrc = boat::canonicalCrc(
        &command, offsetof(boat::WaypointSetPayload, canonicalCrc));
    if (input.corrupt_canonical_crc) command.canonicalCrc ^= 1u;
    beginPending(now_ms, boat::Type::WaypointSet,
                 &command, sizeof(command),
                 command.requestId, command.revision);
    return;
  }

  boat::Type safety_type = boat::Type::Stop;
  switch (input.action) {
    case ProductionAction::Arm: safety_type = boat::Type::Arm; break;
    case ProductionAction::Start: safety_type = boat::Type::StartTest; break;
    case ProductionAction::Stop: safety_type = boat::Type::Stop; break;
    case ProductionAction::Estop: safety_type = boat::Type::Estop; break;
    case ProductionAction::ClearEstop: safety_type = boat::Type::ClearEstop; break;
    case ProductionAction::Disarm: safety_type = boat::Type::Disarm; break;
    default: return;
  }
  boat::CommandPayload command{};
  command.commandId = ++safety_command_id_;
  command.commandType = static_cast<std::uint8_t>(safety_type);
  if (sendCommFrame(now_ms, safety_type, &command, sizeof(command))) {
    ++stats_.safety_commands_sent;
  }
}

void ProductionCommandTelemetrySystem::updateIngressMetrics() {
  const auto& m = ingress_.metrics();
  status_.ingress_new = m.newCommands;
  status_.ingress_applied = m.appliedCommands;
  status_.ingress_rejected = m.rejectedCommands;
  status_.ingress_duplicates = m.duplicates;
  status_.ingress_conflicts = m.protocolConflicts;
  status_.ingress_stale = m.stale;
  status_.ingress_malformed = m.malformed;
}

void ProductionCommandTelemetrySystem::handleControlIngress(
    std::uint32_t now_ms, const boat::Frame& frame) {
  const auto result = ingress_.process(
      frame, status_.safety, controller_,
      static_cast<std::uint64_t>(now_ms) * 1000ULL);
  updateIngressMetrics();
  if (!result.ackGenerated) return;

  boat::ControlCommandAckPayload ack{};
  ack.requestId = result.requestId;
  ack.commandSequence = result.commandSequence;
  ack.commandType = result.commandType;
  ack.disposition = static_cast<std::uint8_t>(result.result.ack);
  ack.safetyState = static_cast<std::uint8_t>(status_.safety);
  ack.controlMode = static_cast<std::uint8_t>(controller_.mode());
  ack.reason = result.result.reason;
  ack.receivedUs = result.receivedUs;
  ack.appliedUs = result.appliedUs;
  ack.canonicalCrc = boat::canonicalCrc(
      &ack, offsetof(boat::ControlCommandAckPayload, canonicalCrc));
  sendControlFrame(now_ms, boat::Type::ControlCommandAck, &ack, sizeof(ack));
}

void ProductionCommandTelemetrySystem::handleWaypointSet(
    std::uint32_t now_ms, const boat::Frame& frame) {
  enum : std::uint8_t { Accepted=0, Rejected=1, Duplicate=2 };
  enum : std::uint8_t {
    None=0, Running=1, Estop=2, Length=3, Crc=4,
    Range=5, Empty=6, Revision=7, State=8
  };

  boat::WaypointAckPayload ack{};
  ack.status = Rejected;
  ack.reason = Length;
  ack.activeIndex = controller_.activeWaypoint();
  ack.count = waypoint_count_;
  if (frame.header.length >= 4) std::memcpy(&ack.requestId, frame.payload, 4);
  if (frame.header.length >= 8) std::memcpy(&ack.revision, frame.payload + 4, 4);

  if (frame.header.length == sizeof(boat::WaypointSetPayload)) {
    boat::WaypointSetPayload request{};
    std::memcpy(&request, frame.payload, sizeof(request));
    ack.requestId = request.requestId;
    ack.revision = request.revision;
    if (request.canonicalCrc != boat::canonicalCrc(
            &request, offsetof(boat::WaypointSetPayload, canonicalCrc))) {
      ack.reason = Crc;
    } else if (status_.safety != production_control::AuthoritativeSafety::Disarmed) {
      ack.reason = status_.safety == production_control::AuthoritativeSafety::Running
                       ? Running
                       : (status_.safety == production_control::AuthoritativeSafety::EStop
                              ? Estop : State);
    } else if (request.revision == waypoint_revision_) {
      ack.status = Duplicate;
      ack.reason = Revision;
    } else if (request.revision < waypoint_revision_) {
      ack.reason = Revision;
    } else if (request.action != 1 || request.count == 0 ||
               request.count > 16 || !std::isfinite(request.reachRadiusM) ||
               request.reachRadiusM < .5f || request.reachRadiusM > 20.0f) {
      ack.reason = request.count == 0 ? Empty : Range;
    } else {
      bool valid = true;
      for (std::uint8_t i = 0; i < request.count; ++i) {
        const auto& p = request.points[i];
        if (!std::isfinite(p.latitudeDeg) || !std::isfinite(p.longitudeDeg) ||
            p.latitudeDeg < -90 || p.latitudeDeg > 90 ||
            p.longitudeDeg < -180 || p.longitudeDeg > 180) {
          valid = false;
          break;
        }
      }
      if (!valid) {
        ack.reason = Range;
      } else {
        waypoint_revision_ = request.revision;
        waypoint_count_ = request.count;
        waypoint_reach_m_ = request.reachRadiusM;
        std::memcpy(waypoint_geo_, request.points,
                    sizeof(boat::WaypointGeo) * request.count);
        controller_.setWaypointReachRadius(
            waypoint_reach_m_, production_control::AuthoritativeSafety::Disarmed);
        production_control::Waypoint local[16]{};
        const double east_scale = 111320.0 *
            std::cos(kOriginLatDeg * 3.14159265358979323846 / 180.0);
        for (std::uint8_t i = 0; i < request.count; ++i) {
          local[i].northM = static_cast<float>(
              (request.points[i].latitudeDeg - kOriginLatDeg) * 111320.0);
          local[i].eastM = static_cast<float>(
              (request.points[i].longitudeDeg - kOriginLonDeg) * east_scale);
        }
        controller_.setWaypoints(local, request.count, request.requestId,
                                 production_control::AuthoritativeSafety::Disarmed);
        ack.status = Accepted;
        ack.reason = None;
        ack.activeIndex = 0;
        ack.count = waypoint_count_;
      }
    }
  }

  ack.canonicalCrc = boat::canonicalCrc(
      &ack, offsetof(boat::WaypointAckPayload, canonicalCrc));
  sendControlFrame(now_ms, boat::Type::WaypointAck, &ack, sizeof(ack));
  status_.waypoint_revision = waypoint_revision_;
  status_.waypoint_count = waypoint_count_;
}

void ProductionCommandTelemetrySystem::sendSafetyAck(
    std::uint32_t now_ms, boat::Type type,
    const boat::CommandPayload& command,
    std::uint8_t disposition,
    std::uint16_t reason) {
  boat::CommandAckPayload ack{};
  ack.commandId = command.commandId;
  ack.commandType = static_cast<std::uint8_t>(type);
  ack.disposition = disposition;
  ack.safetyState = static_cast<std::uint8_t>(status_.safety);
  ack.dryRun = 0;
  ack.receivedUs = static_cast<std::uint64_t>(now_ms) * 1000ULL;
  ack.appliedUs = disposition == 0 ? ack.receivedUs : 0;
  ack.reason = reason;
  sendControlFrame(now_ms, boat::Type::CommandAck, &ack, sizeof(ack));
}

bool ProductionCommandTelemetrySystem::preflightReady(
    std::uint32_t now_ms,
    const ProductionCommandTelemetryInput& input) const {
  if (!status_.host_heartbeat_fresh) return false;
  const auto mode = controller_.mode();
  const bool waypoint_mode = production_control::modeUsesWaypoint(mode);
  const std::uint64_t now_us = static_cast<std::uint64_t>(now_ms) * 1000ULL;
  const bool manual_ok = waypoint_mode ||
      (controller_.manualReceivedUs() &&
       now_us - controller_.manualReceivedUs() <= 500000ULL);
  if (!manual_ok) return false;
  if (mode == production_control::ControlMode::Manual) {
    return controller_.manualOutputMask() != 0;
  }
  if (waypoint_mode) {
    return input.gnss_valid && waypoint_count_ > 0;
  }
  return true;
}

void ProductionCommandTelemetrySystem::applySafetyCommand(
    std::uint32_t now_ms, boat::Type type,
    const boat::Frame& frame) {
  if (frame.header.length != sizeof(boat::CommandPayload)) return;
  boat::CommandPayload command{};
  std::memcpy(&command, frame.payload, sizeof(command));
  if (command.commandType != frame.header.type) {
    sendSafetyAck(now_ms, type, command, 1, 6);
    return;
  }

  std::uint8_t disposition = 0;
  std::uint16_t reason = 0;
  switch (type) {
    case boat::Type::Arm:
      if (status_.safety == production_control::AuthoritativeSafety::Disarmed &&
          preflightReady(now_ms, current_input_)) {
        status_.safety = production_control::AuthoritativeSafety::ArmedIdle;
        status_.stop_reason = production_control::StopReason::None;
      } else {
        disposition = 1;
        reason = 1;
      }
      break;
    case boat::Type::StartTest:
      if (status_.safety == production_control::AuthoritativeSafety::ArmedIdle &&
          preflightReady(now_ms, current_input_)) {
        status_.safety = production_control::AuthoritativeSafety::Running;
        status_.stop_reason = production_control::StopReason::None;
      } else {
        disposition = 1;
        reason = 2;
      }
      break;
    case boat::Type::Stop:
      status_.safety = production_control::AuthoritativeSafety::Disarmed;
      status_.stop_reason = production_control::StopReason::Stop;
      break;
    case boat::Type::Disarm:
      status_.safety = production_control::AuthoritativeSafety::Disarmed;
      status_.stop_reason = production_control::StopReason::None;
      break;
    case boat::Type::Estop:
      status_.safety = production_control::AuthoritativeSafety::EStop;
      status_.stop_reason = production_control::StopReason::EStop;
      break;
    case boat::Type::ClearEstop:
      if (status_.safety == production_control::AuthoritativeSafety::EStop) {
        status_.safety = production_control::AuthoritativeSafety::Disarmed;
        status_.stop_reason = production_control::StopReason::None;
      } else {
        disposition = 1;
        reason = 3;
      }
      break;
    default:
      disposition = 1;
      reason = 4;
      break;
  }
  status_.running = status_.safety == production_control::AuthoritativeSafety::Running;
  sendSafetyAck(now_ms, type, command, disposition, reason);
}

void ProductionCommandTelemetrySystem::processControlRx(
    std::uint32_t now_ms, const boat::Frame& frame) {
  status_.last_control_rx_type = frame.header.type;
  const auto type = static_cast<boat::Type>(frame.header.type);

  if (type == boat::Type::Heartbeat &&
      frame.header.length == sizeof(boat::HeartbeatPayload)) {
    has_host_hb_ = true;
    last_host_hb_rx_ms_ = now_ms;
    return;
  }

  if (type == boat::Type::GnssNavV2 &&
      frame.header.length == sizeof(boat::GnssNavV2Payload)) {
    boat::GnssNavV2Payload nav{};
    std::memcpy(&nav, frame.payload, sizeof(nav));
    const auto expected = boat::canonicalCrc(
        &nav, offsetof(boat::GnssNavV2Payload, canonicalCrc));
    if (nav.canonicalCrc != expected) {
      ++status_.gnss_payload_crc_errors;
    } else {
      ++status_.gnss_nav_rx;
    }
    return;
  }

  if (type == boat::Type::WaypointSet) {
    handleWaypointSet(now_ms, frame);
    return;
  }

  if (type == boat::Type::ControlModeCommand ||
      type == boat::Type::ManualCommand ||
      type == boat::Type::HeadingTarget) {
    handleControlIngress(now_ms, frame);
    return;
  }

  if (type == boat::Type::Arm || type == boat::Type::Disarm ||
      type == boat::Type::StartTest || type == boat::Type::Stop ||
      type == boat::Type::Estop || type == boat::Type::ClearEstop) {
    applySafetyCommand(now_ms, type, frame);
  }
}

void ProductionCommandTelemetrySystem::processCommRx(
    std::uint32_t now_ms, const boat::Frame& frame) {
  has_control_frame_ = true;
  last_control_frame_rx_ms_ = now_ms;
  status_.last_comm_rx_type = frame.header.type;
  const auto type = static_cast<boat::Type>(frame.header.type);

  if (type == boat::Type::Heartbeat &&
      frame.header.length == sizeof(boat::HeartbeatPayload)) {
    ++status_.control_heartbeat_rx;
    return;
  }

  if (type == boat::Type::ControlCommandAck &&
      frame.header.length == sizeof(boat::ControlCommandAckPayload)) {
    boat::ControlCommandAckPayload ack{};
    std::memcpy(&ack, frame.payload, sizeof(ack));
    if (ack.canonicalCrc != boat::canonicalCrc(
            &ack, offsetof(boat::ControlCommandAckPayload, canonicalCrc))) {
      return;
    }
    ++status_.control_command_acks_rx;
    status_.last_control_ack_disposition = ack.disposition;
    status_.last_control_ack_reason = ack.reason;
    if (ack.disposition == static_cast<std::uint8_t>(production_control::Ack::Duplicate)) {
      ++status_.duplicate_acks_rx;
    }
    if (pending_.active && pending_.type != boat::Type::WaypointSet &&
        ack.requestId == pending_.request_id &&
        ack.commandSequence == pending_.sequence_or_revision &&
        ack.commandType == static_cast<std::uint8_t>(pending_.type)) {
      pending_.active = false;
      status_.pending_active = false;
      const bool accepted =
          ack.disposition == static_cast<std::uint8_t>(production_control::Ack::Accepted) ||
          (ack.disposition == static_cast<std::uint8_t>(production_control::Ack::Duplicate) &&
           ack.reason == 0);
      status_.pending_state = accepted ? PendingCommandState::Accepted
                                       : PendingCommandState::Rejected;
      if (accepted) ++stats_.pending_accepted;
      else ++stats_.pending_rejected;
    }
    return;
  }

  if (type == boat::Type::WaypointAck &&
      frame.header.length == sizeof(boat::WaypointAckPayload)) {
    boat::WaypointAckPayload ack{};
    std::memcpy(&ack, frame.payload, sizeof(ack));
    if (ack.canonicalCrc != boat::canonicalCrc(
            &ack, offsetof(boat::WaypointAckPayload, canonicalCrc))) {
      return;
    }
    ++status_.waypoint_acks_rx;
    status_.last_waypoint_ack_status = ack.status;
    status_.last_waypoint_ack_reason = ack.reason;
    if (ack.status == 2) ++status_.duplicate_acks_rx;
    if (pending_.active && pending_.type == boat::Type::WaypointSet &&
        ack.requestId == pending_.request_id &&
        ack.revision == pending_.sequence_or_revision) {
      pending_.active = false;
      status_.pending_active = false;
      const bool accepted = ack.status == 0 || ack.status == 2;
      status_.pending_state = accepted ? PendingCommandState::Accepted
                                       : PendingCommandState::Rejected;
      if (accepted) ++stats_.pending_accepted;
      else ++stats_.pending_rejected;
    }
    return;
  }

  if (type == boat::Type::CommandAck &&
      frame.header.length == sizeof(boat::CommandAckPayload)) {
    ++status_.safety_command_acks_rx;
    return;
  }

  if (type == boat::Type::ControlOutput &&
      frame.header.length == sizeof(boat::ControlOutputPayload)) {
    ++status_.control_output_rx;
    return;
  }

  if (type == boat::Type::ControlSnapshot &&
      frame.header.length == sizeof(boat::ControlSnapshotPayload)) {
    boat::ControlSnapshotPayload snapshot{};
    std::memcpy(&snapshot, frame.payload, sizeof(snapshot));
    ++status_.control_snapshot_rx;
    status_.latest_snapshot_cycle = snapshot.cycle;
    return;
  }

  if (type == boat::Type::InaStatus &&
      frame.header.length == sizeof(boat::InaStatusPayload)) {
    boat::InaStatusPayload power{};
    std::memcpy(&power, frame.payload, sizeof(power));
    ++status_.ina_status_rx;
    status_.latest_ina_bus_v = power.busVoltageV;
    status_.latest_ina_current_a = power.currentA;
    return;
  }

  if (type == boat::Type::VescTelemetry &&
      frame.header.length == sizeof(boat::VescTelemetryPayload)) {
    boat::VescTelemetryPayload motor{};
    std::memcpy(&motor, frame.payload, sizeof(motor));
    ++status_.vesc_telemetry_rx;
    status_.latest_vesc_erpm = motor.erpm;
    return;
  }

  if (type == boat::Type::ActuatorState &&
      frame.header.length == sizeof(boat::ActuatorStatePayload)) {
    boat::ActuatorStatePayload actuators{};
    std::memcpy(&actuators, frame.payload, sizeof(actuators));
    ++status_.actuator_state_rx;
    status_.latest_actuator_pca_ready = actuators.pcaReady;
    status_.latest_actuator_outputs_enabled = actuators.outputsEnabled;
    return;
  }

  if (type == boat::Type::SystemHealth &&
      frame.header.length == sizeof(boat::SystemHealthPayload)) {
    boat::SystemHealthPayload health{};
    std::memcpy(&health, frame.payload, sizeof(health));
    ++status_.system_health_rx;
    status_.latest_health_flags = health.flags;
    status_.latest_health_safety = health.safetyState;
    status_.latest_health_mode = health.controlMode;
  }
}

void ProductionCommandTelemetrySystem::processReceives(std::uint32_t now_ms) {
  for (const auto& wire : uart_.takeForControl()) {
    for (const auto byte : wire.bytes) {
      boat::Frame frame{};
      if (control_decoder_.feed(byte, frame)) processControlRx(now_ms, frame);
    }
  }
  for (const auto& wire : uart_.takeForComm()) {
    for (const auto byte : wire.bytes) {
      boat::Frame frame{};
      if (comm_decoder_.feed(byte, frame)) processCommRx(now_ms, frame);
    }
  }
}

void ProductionCommandTelemetrySystem::updateHealth(std::uint32_t now_ms) {
  status_.comm_link_age_ms = ageOrMax(
      has_control_frame_, now_ms, last_control_frame_rx_ms_);
  status_.host_heartbeat_age_ms = ageOrMax(
      has_host_hb_, now_ms, last_host_hb_rx_ms_);
  status_.comm_link_fresh = has_control_frame_ &&
      status_.comm_link_age_ms <= kCommLinkFreshMs;
  status_.host_heartbeat_fresh = has_host_hb_ &&
      status_.host_heartbeat_age_ms <= kHostHeartbeatTimeoutMs;

  const bool active =
      status_.safety == production_control::AuthoritativeSafety::ArmedIdle ||
      status_.safety == production_control::AuthoritativeSafety::Running;
  if (active && !status_.host_heartbeat_fresh && !status_.failsafe_stop) {
    status_.failsafe_stop = true;
    status_.safety = production_control::AuthoritativeSafety::Disarmed;
    status_.stop_reason = production_control::StopReason::Heartbeat;
    status_.running = false;
    ++stats_.failsafe_stops;
  } else {
    status_.running =
        status_.safety == production_control::AuthoritativeSafety::Running;
  }
}

production_control::SensorInput ProductionCommandTelemetrySystem::makeSensorInput(
    std::uint32_t now_ms,
    const ProductionCommandTelemetryInput& input) const {
  const std::uint64_t now_us = static_cast<std::uint64_t>(now_ms) * 1000ULL;
  production_control::SensorInput sensor{};
  sensor.nowUs = now_us;
  sensor.safety = status_.safety;
  sensor.heartbeat = status_.host_heartbeat_fresh;
  sensor.heartbeatUs = has_host_hb_
      ? static_cast<std::uint64_t>(last_host_hb_rx_ms_) * 1000ULL : 0;
  sensor.imuValid = true;
  sensor.imuUs = now_us;
  sensor.tofValid = true;
  sensor.tofUs = now_us;
  sensor.gnssValid = input.gnss_valid;
  sensor.gnssUs = input.gnss_valid ? now_us : 0;
  sensor.powerValid = true;
  sensor.powerUs = now_us;
  sensor.vescValid = true;
  sensor.vescUs = now_us;
  sensor.vescFault = false;
  sensor.rollRad = 0.02f;
  sensor.pitchRad = -0.01f;
  sensor.yawRad = 0.25f;
  sensor.rollRateRadS = 0.01f;
  sensor.pitchRateRadS = -0.02f;
  sensor.yawRateRadS = 0.03f;
  sensor.tofM = 0.45f;
  sensor.northM = 0.0f;
  sensor.eastM = 0.0f;
  sensor.groundSpeedMps = input.gnss_valid ? 2.5f : 0.0f;
  sensor.courseRad = 0.20f;
  sensor.busVoltageV = 12.2f;
  sensor.currentA = 2.0f;
  sensor.powerW = 24.4f;
  sensor.vescErpm = 1200.0f;
  return sensor;
}

void ProductionCommandTelemetrySystem::sendGnss(
    std::uint32_t now_ms,
    const ProductionCommandTelemetryInput& input) {
  if (now_ms - last_gnss_send_ms_ < kGnssIntervalMs) return;
  last_gnss_send_ms_ = now_ms;
  boat::GnssNavV2Payload nav{};
  nav.navSequence = ++gnss_nav_sequence_;
  nav.fixSequence = ++gnss_fix_sequence_;
  nav.flags = input.gnss_valid ? kAllGnssFlags : 0;
  nav.utcCentiseconds = now_ms / 10;
  nav.latitudeE7 = 364000000;
  nav.longitudeE7 = 1366000000;
  nav.altitudeMm = 10000;
  nav.speedMmPerSec = input.gnss_valid ? 2500 : 0;
  nav.courseE5Deg = 1145916;
  nav.hdopCenti = 80;
  nav.satellites = input.gnss_valid ? 12 : 0;
  nav.fixType = input.gnss_valid ? 3 : 0;
  nav.generatedUs = static_cast<std::uint64_t>(now_ms) * 1000ULL;
  nav.measurementUs = nav.generatedUs;
  nav.sourceBootId = comm_boot_id_;
  nav.canonicalCrc = boat::canonicalCrc(
      &nav, offsetof(boat::GnssNavV2Payload, canonicalCrc));
  if (sendCommFrame(now_ms, boat::Type::GnssNavV2, &nav, sizeof(nav))) {
    ++stats_.gnss_sent;
  }
}

void ProductionCommandTelemetrySystem::sendCommHeartbeat(std::uint32_t now_ms) {
  if (!status_.comm_link_fresh ||
      now_ms - last_comm_hb_send_ms_ < kHeartbeatIntervalMs) return;
  last_comm_hb_send_ms_ = now_ms;
  boat::HeartbeatPayload heartbeat{
      now_ms, comm_frame_sequence_,
      status_.latest_health_safety, 0, 0};
  if (sendCommFrame(now_ms, boat::Type::Heartbeat,
                    &heartbeat, sizeof(heartbeat))) {
    ++stats_.comm_heartbeats_sent;
  }
}

void ProductionCommandTelemetrySystem::sendControlHeartbeat(std::uint32_t now_ms) {
  if (now_ms - last_control_hb_send_ms_ < kHeartbeatIntervalMs) return;
  last_control_hb_send_ms_ = now_ms;
  boat::HeartbeatPayload heartbeat{
      now_ms, control_frame_sequence_,
      static_cast<std::uint8_t>(status_.safety), 0, 0};
  if (sendControlFrame(now_ms, boat::Type::Heartbeat,
                       &heartbeat, sizeof(heartbeat))) {
    ++stats_.control_heartbeats_sent;
  }
}

void ProductionCommandTelemetrySystem::sendTelemetryBurst(
    std::uint32_t now_ms,
    const ProductionCommandTelemetryInput& input) {
  if (now_ms - last_telemetry_send_ms_ < kTelemetryIntervalMs) return;
  last_telemetry_send_ms_ = now_ms;
  const std::uint64_t now_us = static_cast<std::uint64_t>(now_ms) * 1000ULL;
  auto sensor = makeSensorInput(now_ms, input);
  auto output = controller_.step(sensor);

  if (status_.safety == production_control::AuthoritativeSafety::Running) {
    if (output.safetyRequest == production_control::SafetyRequest::Fault) {
      status_.safety = production_control::AuthoritativeSafety::Fault;
      status_.stop_reason = output.reason;
      status_.running = false;
    } else if (output.safetyRequest == production_control::SafetyRequest::Disarm) {
      status_.safety = production_control::AuthoritativeSafety::Disarmed;
      status_.stop_reason = output.reason;
      status_.running = false;
    }
  }

  if (output.physicalGate && status_.running) pwm_writes_ += 3;

  boat::ControlOutputPayload out{};
  out.timestampUs = now_us;
  out.leftFrontWing = output.leftFront;
  out.rightFrontWing = output.rightFront;
  out.rearYaw = output.rearYaw;
  out.propulsion = output.propulsion;
  out.leftPrelimit = output.leftPrelimit;
  out.rightPrelimit = output.rightPrelimit;
  out.rearYawPrelimit = output.rearPrelimit;
  out.propulsionPrelimit = output.propulsionPrelimit;
  out.uHeight = output.uHeight;
  out.uPitch = output.uPitch;
  out.uRoll = output.uRoll;
  out.targetCourseRad = output.targetYaw;
  out.courseErrorRad = output.courseErrorRad;
  out.waypointDistanceM = output.waypointDistanceM;
  out.waypointIndex = output.activeWaypoint;
  out.safety = static_cast<std::uint8_t>(status_.safety);
  out.stopReason = static_cast<std::uint8_t>(status_.stop_reason == production_control::StopReason::None
                                                 ? output.reason : status_.stop_reason);
  out.reservedControl = output.enabledMask;
  out.valid = output.physicalGate ? 1 : 0;
  sendControlFrame(now_ms, boat::Type::ControlOutput, &out, sizeof(out));

  boat::ControlSnapshotPayload snapshot{};
  snapshot.timestampUs = now_us;
  snapshot.cycle = ++telemetry_cycle_;
  snapshot.waypointRevision = waypoint_revision_;
  snapshot.gnssAgeUs = input.gnss_valid ? 0 : 0xffffffffu;
  snapshot.imuAgeUs = 0;
  snapshot.tofAgeUs = 0;
  snapshot.latitudeDeg = kOriginLatDeg;
  snapshot.longitudeDeg = kOriginLonDeg;
  if (waypoint_count_ && output.activeWaypoint < waypoint_count_) {
    snapshot.targetWaypointLatitudeDeg = waypoint_geo_[output.activeWaypoint].latitudeDeg;
    snapshot.targetWaypointLongitudeDeg = waypoint_geo_[output.activeWaypoint].longitudeDeg;
  }
  snapshot.speedMps = sensor.groundSpeedMps;
  snapshot.gnssCourseRad = sensor.courseRad;
  snapshot.localNorthM = sensor.northM;
  snapshot.localEastM = sensor.eastM;
  snapshot.targetBearingRad = output.targetYaw;
  snapshot.courseErrorRad = output.courseErrorRad;
  snapshot.waypointDistanceM = output.waypointDistanceM;
  snapshot.rollRad = sensor.rollRad;
  snapshot.pitchRad = sensor.pitchRad;
  snapshot.yawRad = sensor.yawRad;
  snapshot.rollRateRadS = sensor.rollRateRadS;
  snapshot.pitchRateRadS = sensor.pitchRateRadS;
  snapshot.yawRateRadS = sensor.yawRateRadS;
  snapshot.tofRawMm = 450;
  snapshot.tofFilteredM = sensor.tofM;
  snapshot.heightErrorM = 0.45f - sensor.tofM;
  snapshot.uHeight = output.uHeight;
  snapshot.uPitch = output.uPitch;
  snapshot.uRoll = output.uRoll;
  snapshot.uYaw = output.uYaw;
  snapshot.frontCommon = output.uHeight + output.uPitch;
  snapshot.frontDifferential = output.uRoll;
  snapshot.leftFrontWing = output.leftFront;
  snapshot.rightFrontWing = output.rightFront;
  snapshot.rearYaw = output.rearYaw;
  snapshot.propulsion = output.propulsion;
  snapshot.leftPrelimit = output.leftPrelimit;
  snapshot.rightPrelimit = output.rightPrelimit;
  snapshot.rearYawPrelimit = output.rearPrelimit;
  snapshot.propulsionPrelimit = output.propulsionPrelimit;
  snapshot.gnssValid = input.gnss_valid ? 1 : 0;
  snapshot.imuValid = 1;
  snapshot.tofValid = 1;
  snapshot.heightValid = 1;
  snapshot.waypointReached = output.waypointReached ? 1 : 0;
  snapshot.outputValid = output.physicalGate ? 1 : 0;
  snapshot.state = static_cast<std::uint8_t>(status_.safety);
  snapshot.safetyReason = static_cast<std::uint8_t>(out.stopReason);
  snapshot.mode = static_cast<std::uint8_t>(controller_.mode());
  snapshot.activeWaypoint = output.activeWaypoint;
  sendControlFrame(now_ms, boat::Type::ControlSnapshot,
                   &snapshot, sizeof(snapshot));

  boat::InaStatusPayload power{};
  power.timestampUs = now_us;
  power.ageUs = 0;
  power.busVoltageV = sensor.busVoltageV;
  power.shuntVoltageV = sensor.currentA * 0.002f;
  power.currentA = sensor.currentA;
  power.powerW = sensor.powerW;
  power.valid = 1;
  power.errorCode = 0;
  sendControlFrame(now_ms, boat::Type::InaStatus, &power, sizeof(power));

  boat::VescTelemetryPayload motor{};
  motor.timestampUs = now_us;
  motor.ageUs = 0;
  motor.inputVoltageV = sensor.busVoltageV;
  motor.motorCurrentA = 1.8f;
  motor.inputCurrentA = sensor.currentA;
  motor.duty = output.propulsion * 0.60f;
  motor.erpm = sensor.vescErpm;
  motor.mosTempC = 31.0f;
  motor.motorTempC = 29.0f;
  motor.tachometer = static_cast<std::int32_t>(now_ms * 2);
  motor.valid = 1;
  motor.fault = 0;
  sendControlFrame(now_ms, boat::Type::VescTelemetry, &motor, sizeof(motor));

  boat::ActuatorStatePayload actuators{};
  actuators.timestampUs = now_us;
  actuators.pwmWrites = pwm_writes_;
  actuators.pwmErrors = 0;
  actuators.leftPulseUs = output.enabledMask & production_control::ManualLeft
                              ? servoPulse(output.leftFront) : 0;
  actuators.rightPulseUs = output.enabledMask & production_control::ManualRight
                               ? servoPulse(output.rightFront) : 0;
  actuators.rearPulseUs = output.enabledMask & production_control::ManualRear
                              ? servoPulse(output.rearYaw) : 0;
  actuators.motorRelayEnabled = output.propulsion > 0.0f ? 1 : 0;
  actuators.enabledMask = output.enabledMask;
  actuators.targetDuty = output.propulsion * 0.60f;
  actuators.appliedDuty = actuators.targetDuty;
  actuators.pcaReady = 1;
  actuators.outputsEnabled = status_.running && output.physicalGate ? 1 : 0;
  actuators.safetyState = static_cast<std::uint8_t>(status_.safety);
  actuators.controlMode = static_cast<std::uint8_t>(controller_.mode());
  sendControlFrame(now_ms, boat::Type::ActuatorState,
                   &actuators, sizeof(actuators));

  boat::SystemHealthPayload health{};
  health.timestampUs = now_us;
  health.flags = (1u) | (2u) | (input.gnss_valid ? 4u : 0u) |
                 (8u) | (16u) | (status_.host_heartbeat_fresh ? 32u : 0u);
  health.imuAgeUs = 0;
  health.tofAgeUs = 0;
  health.gnssAgeUs = input.gnss_valid ? 0 : 0xffffffffu;
  health.powerAgeUs = 0;
  health.vescAgeUs = 0;
  health.safetyState = static_cast<std::uint8_t>(status_.safety);
  health.controlMode = static_cast<std::uint8_t>(controller_.mode());
  health.stopReason = static_cast<std::uint8_t>(out.stopReason);
  sendControlFrame(now_ms, boat::Type::SystemHealth, &health, sizeof(health));

  ++stats_.telemetry_bursts_sent;
}

ProductionCommandTelemetryStatus ProductionCommandTelemetrySystem::step(
    std::uint32_t now_ms,
    const ProductionCommandTelemetryInput& input) {
  current_input_ = input;
  uart_.configure(UartDirection::CommToControl,
                  input.comm_to_control_connected,
                  input.comm_to_control_latency_ms);
  uart_.configure(UartDirection::ControlToComm,
                  input.control_to_comm_connected,
                  input.control_to_comm_latency_ms);
  if (input.comm_to_control_fault != InjectedUartFault::None) {
    uart_.injectNext(UartDirection::CommToControl,
                     input.comm_to_control_fault);
  }
  if (input.control_to_comm_fault != InjectedUartFault::None) {
    uart_.injectNext(UartDirection::ControlToComm,
                     input.control_to_comm_fault);
  }

  uart_.advance(now_ms);
  processReceives(now_ms);
  updateHealth(now_ms);
  handleAction(now_ms, input);
  servicePending(now_ms);
  sendGnss(now_ms, input);
  sendCommHeartbeat(now_ms);
  sendTelemetryBurst(now_ms, input);
  sendControlHeartbeat(now_ms);

  status_.control_mode = controller_.mode();
  status_.manual_output_mask = controller_.manualOutputMask();
  status_.waypoint_revision = waypoint_revision_;
  status_.waypoint_count = waypoint_count_;
  updateIngressMetrics();
  return status_;
}

Phase9DecoderStats ProductionCommandTelemetrySystem::decoderStats() const {
  Phase9DecoderStats out{};
  out.comm_crc_errors = comm_decoder_.crcErrors;
  out.comm_cobs_errors = comm_decoder_.cobsErrors;
  out.comm_length_errors = comm_decoder_.lengthErrors;
  out.control_crc_errors = control_decoder_.crcErrors;
  out.control_cobs_errors = control_decoder_.cobsErrors;
  out.control_length_errors = control_decoder_.lengthErrors;
  return out;
}

}  // namespace cores3sim
