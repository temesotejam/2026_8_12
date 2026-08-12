#include "DualControllerSystem.h"

#include <limits>

namespace cores3sim {

DualControllerSystem::DualControllerSystem()
    : uart_(kUartBaud, 8192) {}

void DualControllerSystem::processReceives(std::uint32_t now_ms) {
  for (const auto& packet : uart_.takeForControl()) {
    if (packet.type == LinkPacketType::Heartbeat) {
      has_comm_hb_ = true;
      last_comm_hb_rx_ms_ = now_ms;
    } else if (packet.type == LinkPacketType::GnssNav) {
      ++status_.gnss_received;
    } else if (packet.type == LinkPacketType::Stop) {
      ++status_.stop_received;
      status_.control_running = false;
    }
  }

  for (const auto& packet : uart_.takeForComm()) {
    if (packet.type == LinkPacketType::Heartbeat) {
      has_control_hb_ = true;
      last_control_hb_rx_ms_ = now_ms;
    } else if (packet.type == LinkPacketType::ControlResult) {
      ++status_.results_received;
    }
  }
}

void DualControllerSystem::updateHealth(std::uint32_t now_ms) {
  status_.comm_heartbeat_age_ms =
      has_comm_hb_
          ? now_ms - last_comm_hb_rx_ms_
          : std::numeric_limits<std::uint32_t>::max();
  status_.control_heartbeat_age_ms =
      has_control_hb_
          ? now_ms - last_control_hb_rx_ms_
          : std::numeric_limits<std::uint32_t>::max();

  status_.control_sees_comm_link =
      has_comm_hb_
          ? status_.comm_heartbeat_age_ms <= kHeartbeatTimeoutMs
          : now_ms <= kHeartbeatTimeoutMs;
  status_.comm_sees_control_link =
      has_control_hb_
          ? status_.control_heartbeat_age_ms <= kHeartbeatTimeoutMs
          : now_ms <= kHeartbeatTimeoutMs;

  if (!status_.control_sees_comm_link &&
      !status_.control_failsafe_stop) {
    status_.control_failsafe_stop = true;
    status_.control_running = false;
    ++stats_.failsafe_stops;
  }
}

void DualControllerSystem::sendDue(std::uint32_t now_ms,
                                   const DualControllerInput& input) {
  // STOP is queued before the regular traffic generated on the same tick.
  if (input.send_stop) {
    if (uart_.send(UartDirection::CommToControl,
                   now_ms,
                   LinkPacketType::Stop,
                   next_stop_seq_++,
                   16)) {
      ++stats_.stop_sent;
    }
  }

  if (now_ms == 0 ||
      now_ms - last_gnss_send_ms_ >= kDataIntervalMs) {
    last_gnss_send_ms_ = now_ms;
    if (input.gnss_valid &&
        uart_.send(UartDirection::CommToControl,
                   now_ms,
                   LinkPacketType::GnssNav,
                   next_gnss_seq_++,
                   64)) {
      ++stats_.gnss_sent;
    }
  }

  if (now_ms == 0 ||
      now_ms - last_comm_hb_send_ms_ >= kHeartbeatIntervalMs) {
    last_comm_hb_send_ms_ = now_ms;
    if (uart_.send(UartDirection::CommToControl,
                   now_ms,
                   LinkPacketType::Heartbeat,
                   next_comm_hb_seq_++,
                   16)) {
      ++stats_.comm_heartbeats_sent;
    }
  }

  if (now_ms == 0 ||
      now_ms - last_result_send_ms_ >= kDataIntervalMs) {
    last_result_send_ms_ = now_ms;
    if (uart_.send(UartDirection::ControlToComm,
                   now_ms,
                   LinkPacketType::ControlResult,
                   next_result_seq_++,
                   96)) {
      ++stats_.results_sent;
    }
  }

  if (now_ms == 0 ||
      now_ms - last_control_hb_send_ms_ >= kHeartbeatIntervalMs) {
    last_control_hb_send_ms_ = now_ms;
    if (uart_.send(UartDirection::ControlToComm,
                   now_ms,
                   LinkPacketType::Heartbeat,
                   next_control_hb_seq_++,
                   16)) {
      ++stats_.control_heartbeats_sent;
    }
  }
}

DualControllerStatus DualControllerSystem::step(
    std::uint32_t now_ms,
    const DualControllerInput& input) {
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

}  // namespace cores3sim
