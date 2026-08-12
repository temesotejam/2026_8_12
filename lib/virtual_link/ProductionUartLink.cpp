#include "ProductionUartLink.h"

#include <algorithm>
#include <utility>

namespace cores3sim {

ProductionUartLink::DirectionState& ProductionUartLink::state(
    UartDirection direction) {
  return direction == UartDirection::CommToControl
             ? comm_to_control_
             : control_to_comm_;
}

WireDirectionStats& ProductionUartLink::dirStats(UartDirection direction) {
  return direction == UartDirection::CommToControl
             ? stats_.comm_to_control
             : stats_.control_to_comm;
}

std::uint32_t ProductionUartLink::txDurationMs(std::uint32_t bytes) const {
  if (!bytes || !baud_) return 0;
  const std::uint64_t bits_x_1000 =
      static_cast<std::uint64_t>(bytes) * 10u * 1000u;
  return static_cast<std::uint32_t>((bits_x_1000 + baud_ - 1u) / baud_);
}

void ProductionUartLink::dropAll(UartDirection direction) {
  auto& link = state(direction);
  auto& st = dirStats(direction);
  const auto count = static_cast<std::uint32_t>(link.pending.size());
  st.frames_dropped += count;
  st.dropped_disconnect += count;
  link.pending.clear();
  link.pending_bytes = 0;
  link.tx_busy_until_ms = 0;
}

void ProductionUartLink::configure(UartDirection direction, bool connected,
                                   std::uint32_t latency_ms) {
  auto& link = state(direction);
  if (link.connected && !connected) dropAll(direction);
  link.connected = connected;
  link.latency_ms = latency_ms;
}

void ProductionUartLink::injectNext(UartDirection direction,
                                    InjectedUartFault fault) {
  state(direction).next_fault = fault;
}

void ProductionUartLink::corruptEncodedFrame(std::vector<std::uint8_t>& bytes) {
  if (bytes.size() <= 2) return;
  // Preserve the terminal 0x00 delimiter. Flip a data/code byte in the middle;
  // the production COBS/CRC decoder must reject the resulting wire frame.
  std::size_t index = bytes.size() / 2;
  if (index >= bytes.size() - 1) index = bytes.size() - 2;
  bytes[index] ^= 0x01u;
  if (bytes[index] == 0) bytes[index] = 0x7fu;
}

bool ProductionUartLink::send(UartDirection direction, std::uint32_t now_ms,
                              const std::uint8_t* bytes, std::size_t length) {
  auto& link = state(direction);
  auto& st = dirStats(direction);
  ++st.frames_sent;
  st.bytes_sent += length;

  if (!bytes || !length || !link.connected) {
    ++st.frames_dropped;
    ++st.dropped_disconnect;
    link.next_fault = InjectedUartFault::None;
    return false;
  }

  if (link.pending_bytes + length > capacity_bytes_) {
    ++st.frames_dropped;
    ++st.dropped_overflow;
    link.next_fault = InjectedUartFault::None;
    return false;
  }

  if (link.next_fault == InjectedUartFault::Drop) {
    ++st.frames_dropped;
    ++st.dropped_injected;
    link.next_fault = InjectedUartFault::None;
    return false;
  }

  // A hardware UART framing fault is modeled below the protocol decoder: the
  // affected frame never becomes a valid byte stream for the application.
  if (link.next_fault == InjectedUartFault::Framing) {
    ++st.frames_dropped;
    ++st.framing_errors;
    link.next_fault = InjectedUartFault::None;
    return false;
  }

  WireFrame frame;
  frame.bytes.assign(bytes, bytes + length);
  frame.sent_ms = now_ms;
  if (link.next_fault == InjectedUartFault::Corrupt) {
    corruptEncodedFrame(frame.bytes);
    ++st.corruptions_injected;
  }
  link.next_fault = InjectedUartFault::None;

  const std::uint32_t tx_start = std::max(now_ms, link.tx_busy_until_ms);
  frame.tx_done_ms = tx_start + txDurationMs(static_cast<std::uint32_t>(length));
  frame.deliver_ms = frame.tx_done_ms + link.latency_ms;
  link.tx_busy_until_ms = frame.tx_done_ms;
  link.pending_bytes += static_cast<std::uint32_t>(length);
  st.max_pending_bytes = std::max(st.max_pending_bytes, link.pending_bytes);
  link.pending.push_back(std::move(frame));
  return true;
}

void ProductionUartLink::advance(std::uint32_t now_ms) {
  for (const auto direction : {UartDirection::CommToControl,
                               UartDirection::ControlToComm}) {
    auto& link = state(direction);
    auto& st = dirStats(direction);
    while (!link.pending.empty() && link.pending.front().deliver_ms <= now_ms) {
      WireFrame frame = std::move(link.pending.front());
      link.pending.pop_front();
      const auto size = static_cast<std::uint32_t>(frame.bytes.size());
      link.pending_bytes = link.pending_bytes >= size ? link.pending_bytes - size : 0;
      if (!link.connected) {
        ++st.frames_dropped;
        ++st.dropped_disconnect;
        continue;
      }
      ++st.frames_delivered;
      st.bytes_delivered += size;
      if (direction == UartDirection::CommToControl) {
        to_control_.push_back(std::move(frame));
      } else {
        to_comm_.push_back(std::move(frame));
      }
    }
  }
}

std::vector<WireFrame> ProductionUartLink::takeForControl() {
  auto out = std::move(to_control_);
  to_control_.clear();
  return out;
}

std::vector<WireFrame> ProductionUartLink::takeForComm() {
  auto out = std::move(to_comm_);
  to_comm_.clear();
  return out;
}

}  // namespace cores3sim
