#pragma once

#include <array>
#include <cstdint>

namespace cores3sim {

struct Vl53l5cxInput {
  bool enabled{false};
  bool powered{true};
  bool device_ack{true};
  bool force_reset{false};
  bool ranging_enabled{true};
  bool stall_frames{false};
  std::uint32_t ranging_frequency_hz{10};
  std::uint32_t reinit_delay_ms{500};
  bool auto_reinit{true};

  std::uint16_t default_distance_mm{1000};
  std::uint8_t invalid_zone{255};
  std::uint8_t invalid_status{255};
  std::uint16_t invalid_distance_mm{0};
};

struct Vl53l5cxStatus {
  bool active{false};
  bool initialized{true};
  bool reinitializing{false};
  bool frame_fresh{true};
  std::uint32_t frame_age_ms{0};
  std::uint32_t frame_period_ms{100};

  std::array<std::uint16_t, 64> distance_mm{};
  std::array<std::uint8_t, 64> target_status{};
  std::array<bool, 64> zone_valid{};
  std::uint8_t valid_zones{64};
  std::uint16_t min_distance_mm{0};
};

struct Vl53l5cxStats {
  std::uint32_t resets{0};
  std::uint32_t reinit_attempts{0};
  std::uint32_t reinit_successes{0};
  std::uint32_t frames_delivered{0};
  std::uint32_t stale_events{0};
  std::uint32_t device_nacks{0};
  std::uint32_t invalid_zone_events{0};
};

class VirtualVl53l5cx {
 public:
  static constexpr std::uint32_t kMinFrameStaleMs = 200;
  static constexpr std::uint16_t kMaxDistanceMm = 4000;

  Vl53l5cxStatus update(std::uint32_t now_ms,
                        const Vl53l5cxInput& input,
                        bool bus_ok);

  const Vl53l5cxStats& stats() const { return stats_; }

  static bool targetStatusValid(std::uint8_t status);

 private:
  void clearRuntime();
  void startReinit(std::uint32_t now_ms, std::uint32_t delay_ms);
  void generateFrame(std::uint32_t now_ms, const Vl53l5cxInput& input);

  Vl53l5cxStats stats_{};
  bool initialized_{false};
  bool reinitializing_{false};
  bool has_frame_{false};
  bool reset_latched_{false};
  bool stale_latched_{false};
  bool nack_latched_{false};

  std::uint32_t reinit_ready_at_ms_{0};
  std::uint32_t last_frame_ms_{0};
  std::uint32_t last_generation_ms_{0};

  std::array<std::uint16_t, 64> distance_mm_{};
  std::array<std::uint8_t, 64> target_status_{};
};

}  // namespace cores3sim
