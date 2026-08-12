#pragma once

#include <cstdint>
#include <deque>
#include <vector>

#include "VirtualDuplexUart.h"

namespace cores3sim {

struct WireFrame {
  std::vector<std::uint8_t> bytes{};
  std::uint32_t sent_ms{0};
  std::uint32_t tx_done_ms{0};
  std::uint32_t deliver_ms{0};
};

struct WireDirectionStats {
  std::uint32_t frames_sent{0};
  std::uint32_t frames_delivered{0};
  std::uint32_t frames_dropped{0};
  std::uint32_t dropped_disconnect{0};
  std::uint32_t dropped_injected{0};
  std::uint32_t dropped_overflow{0};
  std::uint32_t framing_errors{0};
  std::uint32_t corruptions_injected{0};
  std::uint64_t bytes_sent{0};
  std::uint64_t bytes_delivered{0};
  std::uint32_t max_pending_bytes{0};
};

struct ProductionUartStats {
  WireDirectionStats comm_to_control{};
  WireDirectionStats control_to_comm{};
};

class ProductionUartLink {
 public:
  explicit ProductionUartLink(std::uint32_t baud = 921600,
                              std::uint32_t pending_capacity_bytes = 16384)
      : baud_(baud), capacity_bytes_(pending_capacity_bytes) {}

  void configure(UartDirection direction, bool connected,
                 std::uint32_t latency_ms);
  void injectNext(UartDirection direction, InjectedUartFault fault);
  bool send(UartDirection direction, std::uint32_t now_ms,
            const std::uint8_t* bytes, std::size_t length);
  bool send(UartDirection direction, std::uint32_t now_ms,
            const std::vector<std::uint8_t>& bytes) {
    return send(direction, now_ms, bytes.data(), bytes.size());
  }
  void advance(std::uint32_t now_ms);

  std::vector<WireFrame> takeForControl();
  std::vector<WireFrame> takeForComm();
  const ProductionUartStats& stats() const { return stats_; }
  std::uint32_t baud() const { return baud_; }

 private:
  struct DirectionState {
    bool connected{true};
    std::uint32_t latency_ms{1};
    std::uint32_t tx_busy_until_ms{0};
    std::uint32_t pending_bytes{0};
    InjectedUartFault next_fault{InjectedUartFault::None};
    std::deque<WireFrame> pending{};
  };

  DirectionState& state(UartDirection direction);
  WireDirectionStats& dirStats(UartDirection direction);
  void dropAll(UartDirection direction);
  std::uint32_t txDurationMs(std::uint32_t bytes) const;
  static void corruptEncodedFrame(std::vector<std::uint8_t>& bytes);

  std::uint32_t baud_{921600};
  std::uint32_t capacity_bytes_{16384};
  DirectionState comm_to_control_{};
  DirectionState control_to_comm_{};
  ProductionUartStats stats_{};
  std::vector<WireFrame> to_control_{};
  std::vector<WireFrame> to_comm_{};
};

}  // namespace cores3sim
