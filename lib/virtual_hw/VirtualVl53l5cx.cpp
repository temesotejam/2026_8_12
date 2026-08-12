#include "VirtualVl53l5cx.h"

#include <algorithm>
#include <limits>

namespace cores3sim {

bool VirtualVl53l5cx::targetStatusValid(std::uint8_t status) {
  // Statuses commonly treated as usable by the VL53L5CX API/application layer.
  return status == 5 || status == 6 || status == 9 || status == 12;
}

void VirtualVl53l5cx::clearRuntime() {
  initialized_ = false;
  reinitializing_ = false;
  has_frame_ = false;
  stale_latched_ = false;
  reinit_ready_at_ms_ = 0;
  last_frame_ms_ = 0;
  last_generation_ms_ = 0;
}

void VirtualVl53l5cx::startReinit(std::uint32_t now_ms,
                                  std::uint32_t delay_ms) {
  reinitializing_ = true;
  initialized_ = false;
  has_frame_ = false;
  stale_latched_ = false;
  reinit_ready_at_ms_ = now_ms + delay_ms;
  ++stats_.reinit_attempts;
}

void VirtualVl53l5cx::generateFrame(std::uint32_t now_ms,
                                    const Vl53l5cxInput& input) {
  distance_mm_.fill(input.default_distance_mm);
  target_status_.fill(5);

  if (input.invalid_zone < 64) {
    distance_mm_[input.invalid_zone] = input.invalid_distance_mm;
    target_status_[input.invalid_zone] = input.invalid_status;
    ++stats_.invalid_zone_events;
  }

  has_frame_ = true;
  last_frame_ms_ = now_ms;
  last_generation_ms_ = now_ms;
  ++stats_.frames_delivered;
}

Vl53l5cxStatus VirtualVl53l5cx::update(std::uint32_t now_ms,
                                       const Vl53l5cxInput& input,
                                       bool bus_ok) {
  Vl53l5cxStatus out;
  out.active = input.enabled;

  const std::uint32_t requested_hz =
      std::max<std::uint32_t>(1, input.ranging_frequency_hz);
  const std::uint32_t effective_hz = std::min<std::uint32_t>(15, requested_hz);
  out.frame_period_ms = std::max<std::uint32_t>(1, 1000 / effective_hz);

  if (!input.enabled) {
    clearRuntime();
    reset_latched_ = false;
    nack_latched_ = false;
    out.initialized = true;
    out.frame_fresh = true;
    out.distance_mm.fill(input.default_distance_mm);
    out.target_status.fill(5);
    out.zone_valid.fill(true);
    out.valid_zones = 64;
    out.min_distance_mm = input.default_distance_mm;
    return out;
  }

  const bool reset_edge = input.force_reset && !reset_latched_;
  reset_latched_ = input.force_reset;

  const bool device_nack = input.powered && bus_ok && !input.device_ack;
  if (device_nack && !nack_latched_) {
    ++stats_.device_nacks;
  }
  nack_latched_ = device_nack;

  const bool device_ok = input.powered && input.device_ack && bus_ok;

  if (!input.powered) {
    clearRuntime();
  } else {
    if (reset_edge) {
      ++stats_.resets;
      clearRuntime();
    }

    if (!device_ok && (initialized_ || reinitializing_)) {
      clearRuntime();
    }

    if (!initialized_ &&
        !reinitializing_ &&
        input.auto_reinit &&
        device_ok) {
      startReinit(now_ms, input.reinit_delay_ms);
    }

    if (reinitializing_ && device_ok && now_ms >= reinit_ready_at_ms_) {
      reinitializing_ = false;
      initialized_ = true;
      has_frame_ = false;
      last_generation_ms_ = now_ms;
      ++stats_.reinit_successes;
    }

    if (initialized_ &&
        device_ok &&
        input.ranging_enabled &&
        !input.stall_frames) {
      const bool due = !has_frame_ ||
                       now_ms - last_generation_ms_ >= out.frame_period_ms;
      if (due) {
        generateFrame(now_ms, input);
      }
    }
  }

  out.initialized = initialized_;
  out.reinitializing = reinitializing_;
  out.distance_mm = distance_mm_;
  out.target_status = target_status_;
  out.zone_valid.fill(false);
  out.valid_zones = 0;
  out.min_distance_mm = 0;

  if (has_frame_) {
    out.frame_age_ms = now_ms - last_frame_ms_;
    const std::uint32_t stale_threshold =
        std::max<std::uint32_t>(kMinFrameStaleMs, out.frame_period_ms * 3);
    out.frame_fresh = out.frame_age_ms <= stale_threshold;

    std::uint16_t min_distance = std::numeric_limits<std::uint16_t>::max();
    for (std::size_t i = 0; i < 64; ++i) {
      const bool valid = targetStatusValid(out.target_status[i]) &&
                         out.distance_mm[i] >= 2 &&
                         out.distance_mm[i] <= kMaxDistanceMm;
      out.zone_valid[i] = valid;
      if (valid) {
        ++out.valid_zones;
        min_distance = std::min(min_distance, out.distance_mm[i]);
      }
    }
    if (out.valid_zones > 0) {
      out.min_distance_mm = min_distance;
    }
  } else {
    out.frame_age_ms = std::numeric_limits<std::uint32_t>::max();
    out.frame_fresh = false;
  }

  const bool stale_now = initialized_ && !out.frame_fresh;
  if (stale_now && !stale_latched_) {
    ++stats_.stale_events;
  }
  stale_latched_ = stale_now;

  return out;
}

}  // namespace cores3sim
