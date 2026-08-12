#include "VirtualDuplexUart.h"

#include <algorithm>

namespace cores3sim {

VirtualDuplexUart::DirectionState& VirtualDuplexUart::state(
    UartDirection direction) {
  return direction == UartDirection::CommToControl
             ? comm_to_control_
             : control_to_comm_;
}

LinkDirectionStats& VirtualDuplexUart::dirStats(UartDirection direction) {
  return direction == UartDirection::CommToControl
             ? stats_.comm_to_control
             : stats_.control_to_comm;
}

std::uint32_t VirtualDuplexUart::txDurationMs(std::uint32_t bytes) const {
  if (bytes == 0 || baud_ == 0) return 0;
  // 8N1 consumes 10 serial bits for each payload byte.
  const std::uint64_t bits_x_1000 =
      static_cast<std::uint64_t>(bytes) * 10u * 1000u;
  return static_cast<std::uint32_t>(
      (bits_x_1000 + baud_ - 1u) / baud_);
}

void VirtualDuplexUart::dropAll(UartDirection direction) {
  auto& link = state(direction);
  auto& st = dirStats(direction);
  const auto count = static_cast<std::uint32_t>(link.pending.size());
  st.packets_dropped += count;
  st.dropped_disconnect += count;
  link.pending.clear();
  link.pending_bytes = 0;
  link.tx_busy_until_ms = 0;
}

void VirtualDuplexUart::configure(UartDirection direction, bool connected,
                                  std::uint32_t latency_ms) {
  auto& link = state(direction);
  if (link.connected && !connected) dropAll(direction);
  link.connected = connected;
  link.latency_ms = latency_ms;
}

void VirtualDuplexUart::injectNext(UartDirection direction,
                                   InjectedUartFault fault) {
  state(direction).next_fault = fault;
}

bool VirtualDuplexUart::send(UartDirection direction, std::uint32_t now_ms,
                             LinkPacketType type, std::uint32_t sequence,
                             std::uint32_t payload_bytes) {
  auto& link = state(direction);
  auto& st = dirStats(direction);
  ++st.packets_sent;
  st.bytes_sent += payload_bytes;

  if (!link.connected) {
    ++st.packets_dropped;
    ++st.dropped_disconnect;
    link.next_fault = InjectedUartFault::None;
    return false;
  }

  if (link.pending_bytes + payload_bytes > capacity_bytes_) {
    ++st.packets_dropped;
    ++st.dropped_overflow;
    link.next_fault = InjectedUartFault::None;
    return false;
  }

  if (link.next_fault == InjectedUartFault::Drop) {
    ++st.packets_dropped;
    ++st.dropped_injected;
    link.next_fault = InjectedUartFault::None;
    return false;
  }

  LinkPacket packet;
  packet.type = type;
  packet.sequence = sequence;
  packet.payload_bytes = payload_bytes;
  packet.sent_ms = now_ms;

  const std::uint32_t tx_start =
      std::max(now_ms, link.tx_busy_until_ms);
  packet.tx_done_ms = tx_start + txDurationMs(payload_bytes);
  packet.deliver_ms = packet.tx_done_ms + link.latency_ms;

  if (link.next_fault == InjectedUartFault::Corrupt) {
    packet.crc_ok = false;
  }
  if (link.next_fault == InjectedUartFault::Framing) {
    packet.framing_ok = false;
  }
  link.next_fault = InjectedUartFault::None;

  link.tx_busy_until_ms = packet.tx_done_ms;
  link.pending_bytes += payload_bytes;
  st.max_pending_bytes =
      std::max(st.max_pending_bytes, link.pending_bytes);
  link.pending.push_back(packet);
  return true;
}

void VirtualDuplexUart::advance(std::uint32_t now_ms) {
  for (const auto direction : {UartDirection::CommToControl,
                               UartDirection::ControlToComm}) {
    auto& link = state(direction);
    auto& st = dirStats(direction);

    while (!link.pending.empty() &&
           link.pending.front().deliver_ms <= now_ms) {
      auto packet = link.pending.front();
      link.pending.pop_front();
      if (link.pending_bytes >= packet.payload_bytes) {
        link.pending_bytes -= packet.payload_bytes;
      } else {
        link.pending_bytes = 0;
      }

      if (!link.connected) {
        ++st.packets_dropped;
        ++st.dropped_disconnect;
      } else if (!packet.framing_ok) {
        ++st.packets_dropped;
        ++st.framing_errors;
      } else if (!packet.crc_ok) {
        ++st.packets_dropped;
        ++st.crc_errors;
      } else {
        ++st.packets_delivered;
        st.bytes_delivered += packet.payload_bytes;
        if (direction == UartDirection::CommToControl) {
          to_control_.push_back(packet);
        } else {
          to_comm_.push_back(packet);
        }
      }
    }
  }
}

std::vector<LinkPacket> VirtualDuplexUart::takeForControl() {
  auto out = to_control_;
  to_control_.clear();
  return out;
}

std::vector<LinkPacket> VirtualDuplexUart::takeForComm() {
  auto out = to_comm_;
  to_comm_.clear();
  return out;
}

}  // namespace cores3sim
