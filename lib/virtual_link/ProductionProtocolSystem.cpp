#include "ProductionProtocolSystem.h"

#include <cstddef>
#include <cstring>
#include <limits>

namespace cores3sim {
namespace {
constexpr std::uint32_t kAllGnssFlags =
    boat::NavFixValid | boat::NavNewFix | boat::NavLatValid |
    boat::NavLonValid | boat::NavAltitudeValid | boat::NavSpeedValid |
    boat::NavCourseValid | boat::NavHdopValid;

std::uint32_t ageOrMax(bool has_value, std::uint32_t now_ms,
                       std::uint32_t timestamp_ms) {
  return has_value ? now_ms - timestamp_ms
                   : std::numeric_limits<std::uint32_t>::max();
}
}

ProductionProtocolSystem::ProductionProtocolSystem()
    : uart_(kUartBaud, 16384) {
  status_.control_safety = ProductionSafety::Disarmed;
  status_.control_running = false;
}

bool ProductionProtocolSystem::sendCommFrame(std::uint32_t now_ms,
                                             boat::Type type,
                                             const void* payload,
                                             std::uint16_t length) {
  if (length > boat::kMaxPayload) return false;
  boat::Header header{boat::kVersion,
                      static_cast<std::uint8_t>(type),
                      length,
                      ++comm_frame_sequence_,
                      comm_boot_id_,
                      static_cast<std::uint64_t>(now_ms) * 1000ULL,
                      0};
  std::uint8_t encoded[boat::kMaxEncoded]{};
  const auto* data = static_cast<const std::uint8_t*>(payload);
  const std::size_t bytes = boat::encode(header, data, encoded, sizeof(encoded));
  if (!bytes) return false;
  const bool queued = uart_.send(UartDirection::CommToControl,
                                 now_ms, encoded, bytes);
  if (queued) ++stats_.comm_frames_sent;
  return queued;
}

bool ProductionProtocolSystem::sendControlFrame(std::uint32_t now_ms,
                                                boat::Type type,
                                                const void* payload,
                                                std::uint16_t length) {
  if (length > boat::kMaxPayload) return false;
  boat::Header header{boat::kVersion,
                      static_cast<std::uint8_t>(type),
                      length,
                      ++control_frame_sequence_,
                      control_boot_id_,
                      static_cast<std::uint64_t>(now_ms) * 1000ULL,
                      0};
  std::uint8_t encoded[boat::kMaxEncoded]{};
  const auto* data = static_cast<const std::uint8_t*>(payload);
  const std::size_t bytes = boat::encode(header, data, encoded, sizeof(encoded));
  if (!bytes) return false;
  const bool queued = uart_.send(UartDirection::ControlToComm,
                                 now_ms, encoded, bytes);
  if (queued) ++stats_.control_frames_sent;
  return queued;
}

void ProductionProtocolSystem::sendSafetyCommand(std::uint32_t now_ms,
                                                 boat::Type type) {
  boat::CommandPayload command{};
  command.commandId = ++safety_command_id_;
  command.commandType = static_cast<std::uint8_t>(type);
  if (sendCommFrame(now_ms, type, &command, sizeof(command))) {
    ++stats_.safety_commands_sent;
  }
}

void ProductionProtocolSystem::sendSafetyAck(
    std::uint32_t now_ms, const boat::CommandPayload& command,
    boat::Type type, std::uint8_t disposition, std::uint16_t reason) {
  boat::CommandAckPayload ack{};
  ack.commandId = command.commandId;
  ack.commandType = static_cast<std::uint8_t>(type);
  ack.disposition = disposition;
  ack.safetyState = static_cast<std::uint8_t>(status_.control_safety);
  ack.dryRun = 0;
  ack.receivedUs = static_cast<std::uint64_t>(now_ms) * 1000ULL;
  ack.appliedUs = disposition == 0 ? ack.receivedUs : 0;
  ack.reason = reason;
  if (sendControlFrame(now_ms, boat::Type::CommandAck,
                       &ack, sizeof(ack))) {
    ++stats_.command_acks_sent;
  }
}

void ProductionProtocolSystem::applySafetyCommand(
    std::uint32_t now_ms, boat::Type type,
    const boat::CommandPayload& command) {
  std::uint8_t disposition = 0;
  std::uint16_t reason = 0;

  switch (type) {
    case boat::Type::Stop:
      ++status_.stop_received;
      status_.control_safety = ProductionSafety::Disarmed;
      status_.stop_reason = ProductionStopReason::Stop;
      break;
    case boat::Type::Estop:
      ++status_.estop_received;
      status_.control_safety = ProductionSafety::EStop;
      status_.stop_reason = ProductionStopReason::EStop;
      break;
    case boat::Type::ClearEstop:
      ++status_.clear_estop_received;
      if (status_.control_safety == ProductionSafety::EStop) {
        status_.control_safety = ProductionSafety::Disarmed;
        status_.stop_reason = ProductionStopReason::None;
      } else {
        disposition = 1;
        reason = 1;
      }
      break;
    case boat::Type::Disarm:
      ++status_.disarm_received;
      if (status_.control_safety != ProductionSafety::EStop) {
        status_.control_safety = ProductionSafety::Disarmed;
        status_.stop_reason = ProductionStopReason::None;
      } else {
        disposition = 1;
        reason = 2;
      }
      break;
    case boat::Type::Arm:
      ++status_.arm_received;
      if (status_.control_safety == ProductionSafety::Disarmed &&
          status_.control_host_heartbeat_fresh) {
        status_.control_safety = ProductionSafety::Armed;
        status_.stop_reason = ProductionStopReason::None;
      } else {
        disposition = 1;
        reason = 3;
      }
      break;
    case boat::Type::StartTest:
      ++status_.start_received;
      if (status_.control_safety == ProductionSafety::Armed &&
          status_.control_host_heartbeat_fresh) {
        status_.control_safety = ProductionSafety::Running;
        status_.stop_reason = ProductionStopReason::None;
      } else {
        disposition = 1;
        reason = 4;
      }
      break;
    default:
      disposition = 1;
      reason = 5;
      break;
  }

  status_.control_running =
      status_.control_safety == ProductionSafety::Running;
  sendSafetyAck(now_ms, command, type, disposition, reason);
}

void ProductionProtocolSystem::processControlFrame(
    std::uint32_t now_ms, const boat::Frame& frame) {
  status_.last_control_rx_type = frame.header.type;
  status_.last_control_rx_sequence = frame.header.sequence;
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
    const std::uint32_t expected = boat::canonicalCrc(
        &nav, offsetof(boat::GnssNavV2Payload, canonicalCrc));
    if (nav.canonicalCrc != expected) {
      ++status_.gnss_canonical_crc_errors;
      return;
    }
    ++status_.gnss_received;
    return;
  }

  const bool safety_type =
      type == boat::Type::Arm || type == boat::Type::Disarm ||
      type == boat::Type::StartTest || type == boat::Type::Stop ||
      type == boat::Type::Estop || type == boat::Type::ClearEstop;
  if (!safety_type || frame.header.length != sizeof(boat::CommandPayload)) {
    return;
  }

  boat::CommandPayload command{};
  std::memcpy(&command, frame.payload, sizeof(command));
  if (command.commandType != frame.header.type) {
    sendSafetyAck(now_ms, command, type, 1, 6);
    return;
  }
  applySafetyCommand(now_ms, type, command);
}

void ProductionProtocolSystem::processCommFrame(
    std::uint32_t now_ms, const boat::Frame& frame) {
  has_control_frame_ = true;
  last_control_frame_rx_ms_ = now_ms;
  ++status_.control_frames_received;
  status_.last_comm_rx_type = frame.header.type;
  status_.last_comm_rx_sequence = frame.header.sequence;
  const auto type = static_cast<boat::Type>(frame.header.type);

  if (type == boat::Type::ControlOutput &&
      frame.header.length == sizeof(boat::ControlOutputPayload)) {
    boat::ControlOutputPayload output{};
    std::memcpy(&output, frame.payload, sizeof(output));
    comm_cached_safety_ = output.safety;
    return;
  }

  if (type == boat::Type::Heartbeat &&
      frame.header.length == sizeof(boat::HeartbeatPayload)) {
    boat::HeartbeatPayload hb{};
    std::memcpy(&hb, frame.payload, sizeof(hb));
    comm_cached_safety_ = hb.safetyState;
    return;
  }

  if (type == boat::Type::CommandAck &&
      frame.header.length == sizeof(boat::CommandAckPayload)) {
    boat::CommandAckPayload ack{};
    std::memcpy(&ack, frame.payload, sizeof(ack));
    ++status_.command_acks_received;
    status_.last_ack_command_type = ack.commandType;
    status_.last_ack_disposition = ack.disposition;
    comm_cached_safety_ = ack.safetyState;
  }
}

void ProductionProtocolSystem::processReceives(std::uint32_t now_ms) {
  for (const auto& wire : uart_.takeForControl()) {
    for (const std::uint8_t byte : wire.bytes) {
      boat::Frame frame{};
      if (control_decoder_.feed(byte, frame)) {
        processControlFrame(now_ms, frame);
      }
    }
  }

  for (const auto& wire : uart_.takeForComm()) {
    for (const std::uint8_t byte : wire.bytes) {
      boat::Frame frame{};
      if (comm_decoder_.feed(byte, frame)) {
        processCommFrame(now_ms, frame);
      }
    }
  }
}

void ProductionProtocolSystem::updateHealth(std::uint32_t now_ms) {
  status_.comm_link_age_ms = ageOrMax(has_control_frame_, now_ms,
                                      last_control_frame_rx_ms_);
  status_.host_heartbeat_age_ms = ageOrMax(has_host_hb_, now_ms,
                                           last_host_hb_rx_ms_);
  status_.comm_link_fresh =
      has_control_frame_ && status_.comm_link_age_ms <= kCommLinkFreshMs;
  status_.control_host_heartbeat_fresh =
      has_host_hb_ && status_.host_heartbeat_age_ms <= kHostHeartbeatTimeoutMs;

  const bool active = status_.control_safety == ProductionSafety::Armed ||
                      status_.control_safety == ProductionSafety::Running;
  if (active && !status_.control_host_heartbeat_fresh &&
      !status_.control_failsafe_stop) {
    status_.control_failsafe_stop = true;
    status_.control_safety = ProductionSafety::Disarmed;
    status_.stop_reason = ProductionStopReason::Heartbeat;
    status_.control_running = false;
    ++stats_.failsafe_stops;
  } else {
    status_.control_running =
        status_.control_safety == ProductionSafety::Running;
  }
}

void ProductionProtocolSystem::sendDue(
    std::uint32_t now_ms, const ProductionProtocolInput& input) {
  // Match the communication-node safety path: a CommandPayload is wrapped in
  // the safety Type itself. Only one safety action is emitted per simulator tick.
  if (input.send_estop) {
    sendSafetyCommand(now_ms, boat::Type::Estop);
  } else if (input.send_stop) {
    sendSafetyCommand(now_ms, boat::Type::Stop);
  } else if (input.send_clear_estop) {
    sendSafetyCommand(now_ms, boat::Type::ClearEstop);
  } else if (input.send_disarm) {
    sendSafetyCommand(now_ms, boat::Type::Disarm);
  } else if (input.send_arm) {
    sendSafetyCommand(now_ms, boat::Type::Arm);
  } else if (input.send_start) {
    sendSafetyCommand(now_ms, boat::Type::StartTest);
  }

  if (now_ms == 0 || now_ms - last_gnss_send_ms_ >= kGnssIntervalMs) {
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
    nav.courseE5Deg = 9000000;
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

  // Production communication firmware only sends host heartbeat while it is
  // itself receiving fresh control-node telemetry.
  if (status_.comm_link_fresh &&
      (now_ms == 0 || now_ms - last_comm_hb_send_ms_ >= kHeartbeatIntervalMs)) {
    last_comm_hb_send_ms_ = now_ms;
    boat::HeartbeatPayload hb{now_ms, comm_frame_sequence_,
                              comm_cached_safety_, 0, 0};
    if (sendCommFrame(now_ms, boat::Type::Heartbeat, &hb, sizeof(hb))) {
      ++stats_.comm_heartbeats_sent;
    }
  }

  if (now_ms == 0 ||
      now_ms - last_control_telemetry_send_ms_ >= kControlTelemetryIntervalMs) {
    last_control_telemetry_send_ms_ = now_ms;
    boat::ControlOutputPayload output{};
    output.timestampUs = static_cast<std::uint64_t>(now_ms) * 1000ULL;
    output.safety = static_cast<std::uint8_t>(status_.control_safety);
    output.stopReason = static_cast<std::uint8_t>(status_.stop_reason);
    output.valid = status_.control_running ? 1 : 0;
    if (sendControlFrame(now_ms, boat::Type::ControlOutput,
                         &output, sizeof(output))) {
      ++stats_.control_outputs_sent;
    }
  }

  if (now_ms == 0 ||
      now_ms - last_control_hb_send_ms_ >= kHeartbeatIntervalMs) {
    last_control_hb_send_ms_ = now_ms;
    boat::HeartbeatPayload hb{
        now_ms, control_frame_sequence_,
        static_cast<std::uint8_t>(status_.control_safety), 0, 0};
    if (sendControlFrame(now_ms, boat::Type::Heartbeat, &hb, sizeof(hb))) {
      ++stats_.control_heartbeats_sent;
    }
  }
}

ProductionProtocolStatus ProductionProtocolSystem::step(
    std::uint32_t now_ms, const ProductionProtocolInput& input) {
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
  sendDue(now_ms, input);
  return status_;
}

ProtocolDecoderStats ProductionProtocolSystem::decoderStats() const {
  ProtocolDecoderStats out{};
  out.comm_crc_errors = comm_decoder_.crcErrors;
  out.comm_cobs_errors = comm_decoder_.cobsErrors;
  out.comm_length_errors = comm_decoder_.lengthErrors;
  out.control_crc_errors = control_decoder_.crcErrors;
  out.control_cobs_errors = control_decoder_.cobsErrors;
  out.control_length_errors = control_decoder_.lengthErrors;
  return out;
}

}  // namespace cores3sim
