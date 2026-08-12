#pragma once

#include <cstdint>

namespace cores3sim {

struct Bno08xInput {
  bool enabled{false};
  bool powered{true};
  bool force_reset{false};
  bool reports_enabled{true};
  bool stall_reports{false};
  std::uint32_t report_interval_ms{20};
  std::uint32_t reinit_delay_ms{300};
  bool auto_reinit{true};
  float pitch_deg{0.0f};
};

struct Bno08xStatus {
  bool active{false};
  bool initialized{true};
  bool reinitializing{false};
  bool report_fresh{true};
  std::uint32_t report_age_ms{0};
  float pitch_deg{0.0f};
};

struct Bno08xStats {
  std::uint32_t resets{0};
  std::uint32_t reinit_attempts{0};
  std::uint32_t reinit_successes{0};
  std::uint32_t reports_delivered{0};
  std::uint32_t stale_events{0};
};

class VirtualBno08x {
 public:
  static constexpr std::uint32_t kMinReportStaleMs = 100;

  Bno08xStatus update(std::uint32_t now_ms,
                      const Bno08xInput& input,
                      bool bus_ok);

  const Bno08xStats& stats() const { return stats_; }

 private:
  void clearRuntime();
  void startReinit(std::uint32_t now_ms, std::uint32_t delay_ms);

  Bno08xStats stats_{};

  bool initialized_{false};
  bool reinitializing_{false};
  bool has_report_{false};
  bool reset_latched_{false};
  bool stale_latched_{false};

  std::uint32_t reinit_ready_at_ms_{0};
  std::uint32_t last_report_ms_{0};
  std::uint32_t last_generation_ms_{0};
  float last_pitch_deg_{0.0f};
};

}  // namespace cores3sim
