#pragma once

#include <cstdint>
#include <deque>
#include <vector>

namespace cores3sim {

enum class UartDirection { CommToControl, ControlToComm };
enum class LinkPacketType { Heartbeat, GnssNav, ControlResult, Stop };
enum class InjectedUartFault { None, Drop, Corrupt, Framing };

struct LinkPacket {
  LinkPacketType type{LinkPacketType::Heartbeat};
  std::uint32_t sequence{0};
  std::uint32_t payload_bytes{0};
  std::uint32_t sent_ms{0};
  std::uint32_t tx_done_ms{0};
  std::uint32_t deliver_ms{0};
  bool crc_ok{true};
  bool framing_ok{true};
};

struct LinkDirectionStats {
  std::uint32_t packets_sent{0};
  std::uint32_t packets_delivered{0};
  std::uint32_t packets_dropped{0};
  std::uint32_t dropped_disconnect{0};
  std::uint32_t dropped_injected{0};
  std::uint32_t dropped_overflow{0};
  std::uint32_t crc_errors{0};
  std::uint32_t framing_errors{0};
  std::uint64_t bytes_sent{0};
  std::uint64_t bytes_delivered{0};
  std::uint32_t max_pending_bytes{0};
};

struct DuplexUartStats {
  LinkDirectionStats comm_to_control{};
  LinkDirectionStats control_to_comm{};
};

class VirtualDuplexUart {
 public:
  explicit VirtualDuplexUart(std::uint32_t baud = 921600,
                             std::uint32_t pending_capacity_bytes = 8192)
      : baud_(baud), capacity_bytes_(pending_capacity_bytes) {}

  void configure(UartDirection direction, bool connected,
                 std::uint32_t latency_ms);
  void injectNext(UartDirection direction, InjectedUartFault fault);
  bool send(UartDirection direction, std::uint32_t now_ms,
            LinkPacketType type, std::uint32_t sequence,
            std::uint32_t payload_bytes);
  void advance(std::uint32_t now_ms);

  std::vector<LinkPacket> takeForControl();
  std::vector<LinkPacket> takeForComm();
  const DuplexUartStats& stats() const { return stats_; }
  std::uint32_t baud() const { return baud_; }

 private:
  struct DirectionState {
    bool connected{true};
    std::uint32_t latency_ms{1};
    std::uint32_t tx_busy_until_ms{0};
    std::uint32_t pending_bytes{0};
    InjectedUartFault next_fault{InjectedUartFault::None};
    std::deque<LinkPacket> pending{};
  };

  DirectionState& state(UartDirection direction);
  LinkDirectionStats& dirStats(UartDirection direction);
  void dropAll(UartDirection direction);
  std::uint32_t txDurationMs(std::uint32_t bytes) const;

  std::uint32_t baud_{921600};
  std::uint32_t capacity_bytes_{8192};
  DirectionState comm_to_control_{};
  DirectionState control_to_comm_{};
  DuplexUartStats stats_{};
  std::vector<LinkPacket> to_control_{};
  std::vector<LinkPacket> to_comm_{};
};

}  // namespace cores3sim
