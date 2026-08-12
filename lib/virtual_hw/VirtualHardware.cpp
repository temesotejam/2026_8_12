#include "VirtualHardware.h"

#include <cstdlib>
#include <limits>

namespace cores3sim {

void VirtualHardware::apply(std::uint32_t now_ms, const VirtualHwInput& input) {
  input_ = input;
  uart_frame_ok_ = true;

  if (!input_.uart_connected && was_uart_connected_) {
    stats_.uart_frames_dropped += static_cast<std::uint32_t>(pending_gnss_.size());
    pending_gnss_.clear();
  }
  was_uart_connected_ = input_.uart_connected;

  if (input_.gnss_source_valid) {
    ++stats_.uart_frames_injected;
    if (!input_.uart_connected) {
      ++stats_.uart_frames_dropped;
    } else {
      const bool bad_frame = input_.uart_corrupt_byte ||
                             input_.uart_crc_error ||
                             input_.uart_framing_error;
      if (input_.uart_corrupt_byte) ++stats_.uart_corrupt_bytes;
      if (input_.uart_corrupt_byte || input_.uart_crc_error) {
        ++stats_.uart_crc_errors;
      }
      if (input_.uart_framing_error) ++stats_.uart_framing_errors;

      if (bad_frame) {
        uart_frame_ok_ = false;
        ++stats_.uart_frames_dropped;
      } else {
        PendingGnss p;
        p.deliver_at_ms = now_ms + input_.uart_latency_ms;
        p.sequence = next_gnss_sequence_++;
        pending_gnss_.push_back(p);
      }
    }
  }
}

void VirtualHardware::deliverPending(std::uint32_t now_ms) {
  auto it = pending_gnss_.begin();
  while (it != pending_gnss_.end()) {
    if (it->deliver_at_ms <= now_ms) {
      has_gnss_ = true;
      last_gnss_ms_ = now_ms;
      ++stats_.uart_frames_delivered;
      it = pending_gnss_.erase(it);
    } else {
      ++it;
    }
  }
}

SensorSample VirtualHardware::sample(std::uint32_t now_ms) {
  SensorSample out;
  out.battery_v = input_.battery_v;

  ++stats_.i2c_reads;
  out.i2c_nack = input_.i2c_connected && input_.i2c_nack;
  if (out.i2c_nack) ++stats_.i2c_nacks;

  out.i2c_ok = input_.i2c_connected &&
               !out.i2c_nack &&
               input_.i2c_latency_ms <= kI2cTimeoutMs;
  if (!out.i2c_ok && !out.i2c_nack) ++stats_.i2c_timeouts;

  out.imu_ok = out.i2c_ok && input_.imu_online;
  if (out.imu_ok) last_pitch_deg_ = input_.pitch_deg;
  out.pitch_deg = last_pitch_deg_;

  deliverPending(now_ms);
  out.uart_ok = input_.uart_connected;
  out.uart_frame_ok = uart_frame_ok_;

  if (has_gnss_) {
    out.gnss_age_ms = now_ms - last_gnss_ms_;
    out.gnss_ok = out.gnss_age_ms <= kGnssStaleMs;
  } else {
    out.gnss_age_ms = std::numeric_limits<std::uint32_t>::max();
    out.gnss_ok = false;
  }

  out.loop_jitter_ms = input_.loop_jitter_ms;
  const std::uint32_t abs_jitter =
      static_cast<std::uint32_t>(std::abs(input_.loop_jitter_ms));
  out.timing_ok = abs_jitter <= kLoopJitterWarnMs;
  if (!out.timing_ok) ++stats_.timing_jitter_events;
  if (abs_jitter > stats_.max_abs_jitter_ms) {
    stats_.max_abs_jitter_ms = abs_jitter;
  }

  ++stats_.sd_write_attempts;
  const bool sd_timeout = input_.sd_connected &&
                          input_.sd_latency_ms > kSdTimeoutMs;
  out.sd_ok = input_.sd_connected && !input_.sd_fail_write && !sd_timeout;
  if (!out.sd_ok) ++stats_.sd_write_failures;
  if (sd_timeout) ++stats_.sd_write_timeouts;

  return out;
}

}  // namespace cores3sim
