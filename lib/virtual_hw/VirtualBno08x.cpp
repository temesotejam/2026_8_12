#include "VirtualBno08x.h"

#include <algorithm>
#include <limits>

namespace cores3sim {

void VirtualBno08x::clearRuntime() {
  initialized_ = false;
  reinitializing_ = false;
  has_report_ = false;
  stale_latched_ = false;
  reinit_ready_at_ms_ = 0;
  last_report_ms_ = 0;
  last_generation_ms_ = 0;
}

void VirtualBno08x::startReinit(std::uint32_t now_ms,
                                std::uint32_t delay_ms) {
  reinitializing_ = true;
  initialized_ = false;
  has_report_ = false;
  stale_latched_ = false;
  reinit_ready_at_ms_ = now_ms + delay_ms;
  ++stats_.reinit_attempts;
}

Bno08xStatus VirtualBno08x::update(std::uint32_t now_ms,
                                   const Bno08xInput& input,
                                   bool bus_ok) {
  Bno08xStatus out;
  out.active = input.enabled;

  if (!input.enabled) {
    clearRuntime();
    reset_latched_ = false;
    out.initialized = true;
    out.report_fresh = true;
    out.pitch_deg = input.pitch_deg;
    return out;
  }

  const bool reset_edge = input.force_reset && !reset_latched_;
  reset_latched_ = input.force_reset;

  if (!input.powered) {
    clearRuntime();
  } else {
    if (reset_edge) {
      ++stats_.resets;
      clearRuntime();
    }

    // Host-side reconfiguration cannot complete while the I2C path is unusable.
    // Cancel the attempt and restart it after the bus recovers.
    if (reinitializing_ && !bus_ok) {
      reinitializing_ = false;
      reinit_ready_at_ms_ = 0;
    }

    if (!initialized_ &&
        !reinitializing_ &&
        input.auto_reinit &&
        bus_ok) {
      startReinit(now_ms, input.reinit_delay_ms);
    }

    if (reinitializing_ &&
        bus_ok &&
        now_ms >= reinit_ready_at_ms_) {
      reinitializing_ = false;
      initialized_ = true;
      has_report_ = false;
      last_generation_ms_ = now_ms;
      ++stats_.reinit_successes;
    }

    const std::uint32_t interval =
        std::max<std::uint32_t>(1, input.report_interval_ms);

    if (initialized_ &&
        bus_ok &&
        input.reports_enabled &&
        !input.stall_reports) {
      const bool due = !has_report_ ||
                       now_ms - last_generation_ms_ >= interval;
      if (due) {
        last_generation_ms_ = now_ms;
        last_report_ms_ = now_ms;
        last_pitch_deg_ = input.pitch_deg;
        has_report_ = true;
        ++stats_.reports_delivered;
      }
    }
  }

  out.initialized = initialized_;
  out.reinitializing = reinitializing_;
  out.pitch_deg = last_pitch_deg_;

  if (has_report_) {
    out.report_age_ms = now_ms - last_report_ms_;
    const std::uint32_t interval =
        std::max<std::uint32_t>(1, input.report_interval_ms);
    const std::uint32_t stale_threshold =
        std::max<std::uint32_t>(kMinReportStaleMs, interval * 5);
    out.report_fresh = out.report_age_ms <= stale_threshold;
  } else {
    out.report_age_ms = std::numeric_limits<std::uint32_t>::max();
    out.report_fresh = false;
  }

  const bool stale_now = initialized_ && !out.report_fresh;
  if (stale_now && !stale_latched_) {
    ++stats_.stale_events;
  }
  stale_latched_ = stale_now;

  return out;
}

}  // namespace cores3sim
