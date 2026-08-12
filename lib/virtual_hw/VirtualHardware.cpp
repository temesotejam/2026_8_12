#include "VirtualHardware.h"

#include <cstdlib>
#include <limits>

namespace cores3sim {

void VirtualHardware::apply(std::uint32_t now_ms, const VirtualHwInput& input) {
  input_ = input;
  uart_frame_ok_ = true;

  if (!input_.uart_connected && was_uart_connected_) {
    stats_.uart_frames_dropped +=
        static_cast<std::uint32_t>(pending_gnss_.size());
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

  if (input_.bno_model_enabled) {
    Bno08xInput bno_input;
    bno_input.enabled = true;
    bno_input.powered = input_.imu_online;
    bno_input.force_reset = input_.bno_force_reset;
    bno_input.reports_enabled = input_.bno_reports_enabled;
    bno_input.stall_reports = input_.bno_stall_reports;
    bno_input.report_interval_ms = input_.bno_report_interval_ms;
    bno_input.reinit_delay_ms = input_.bno_reinit_delay_ms;
    bno_input.auto_reinit = input_.bno_auto_reinit;
    bno_input.pitch_deg = input_.pitch_deg;

    const auto bno = bno_.update(now_ms, bno_input, out.i2c_ok);
    out.bno_model_active = true;
    out.bno_initialized = bno.initialized;
    out.bno_report_fresh = bno.report_fresh;
    out.bno_reinitializing = bno.reinitializing;
    out.bno_report_age_ms = bno.report_age_ms;
    out.imu_ok = out.i2c_ok &&
                 input_.imu_online &&
                 bno.initialized &&
                 bno.report_fresh;
    out.pitch_deg = bno.pitch_deg;
  } else {
    out.imu_ok = out.i2c_ok && input_.imu_online;
    if (out.imu_ok) last_pitch_deg_ = input_.pitch_deg;
    out.pitch_deg = last_pitch_deg_;

    Bno08xInput disabled_bno;
    disabled_bno.enabled = false;
    disabled_bno.pitch_deg = out.pitch_deg;
    (void)bno_.update(now_ms, disabled_bno, out.i2c_ok);
  }

  Ina226Input ina_input;
  ina_input.enabled = input_.ina_model_enabled;
  ina_input.powered = input_.ina_powered;
  ina_input.device_ack = input_.ina_device_ack;
  ina_input.force_reset = input_.ina_force_reset;
  ina_input.conversion_enabled = input_.ina_conversion_enabled;
  ina_input.stall_conversions = input_.ina_stall_conversions;
  ina_input.calibration_programmed = input_.ina_calibration_programmed;
  ina_input.force_math_overflow = input_.ina_force_math_overflow;
  ina_input.bus_voltage_v = input_.ina_bus_voltage_v;
  ina_input.current_a = input_.ina_current_a;
  ina_input.shunt_ohm = input_.ina_shunt_ohm;
  ina_input.current_lsb_a = input_.ina_current_lsb_a;
  ina_input.shunt_conversion_us = input_.ina_shunt_conversion_us;
  ina_input.bus_conversion_us = input_.ina_bus_conversion_us;
  ina_input.averages = input_.ina_averages;
  ina_input.reinit_delay_ms = input_.ina_reinit_delay_ms;
  ina_input.auto_reinit = input_.ina_auto_reinit;

  const auto ina = ina_.update(now_ms, ina_input, out.i2c_ok);
  out.ina_model_active = ina.active;
  out.ina_device_ok = ina.device_ok;
  out.ina_initialized = ina.initialized;
  out.ina_reinitializing = ina.reinitializing;
  out.ina_conversion_fresh = ina.conversion_fresh;
  out.ina_calibration_ok = ina.calibration_ok;
  out.ina_range_ok = ina.range_ok;
  out.ina_math_overflow = ina.math_overflow;
  out.ina_measurement_age_ms = ina.measurement_age_ms;
  out.ina_bus_voltage_v = ina.bus_voltage_v;
  out.ina_current_a = ina.current_a;
  out.ina_power_w = ina.power_w;
  out.ina_shunt_voltage_v = ina.shunt_voltage_v;

  Vl53l5cxInput tof_input;
  tof_input.enabled = input_.tof_model_enabled;
  tof_input.powered = input_.tof_powered;
  tof_input.device_ack = input_.tof_device_ack;
  tof_input.force_reset = input_.tof_force_reset;
  tof_input.ranging_enabled = input_.tof_ranging_enabled;
  tof_input.stall_frames = input_.tof_stall_frames;
  tof_input.ranging_frequency_hz = input_.tof_ranging_frequency_hz;
  tof_input.reinit_delay_ms = input_.tof_reinit_delay_ms;
  tof_input.auto_reinit = input_.tof_auto_reinit;
  tof_input.default_distance_mm = input_.tof_default_distance_mm;
  tof_input.invalid_zone = input_.tof_invalid_zone;
  tof_input.invalid_status = input_.tof_invalid_status;
  tof_input.invalid_distance_mm = input_.tof_invalid_distance_mm;

  const auto tof = tof_.update(now_ms, tof_input, out.i2c_ok);
  out.tof_model_active = tof.active;
  out.tof_device_ok = !tof.active ||
                      (input_.tof_powered && input_.tof_device_ack && out.i2c_ok);
  out.tof_initialized = tof.initialized;
  out.tof_reinitializing = tof.reinitializing;
  out.tof_frame_fresh = tof.frame_fresh;
  out.tof_frame_age_ms = tof.frame_age_ms;
  out.tof_valid_zones = tof.valid_zones;
  out.tof_min_distance_mm = tof.min_distance_mm;

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
  out.sd_ok = input_.sd_connected &&
              !input_.sd_fail_write &&
              !sd_timeout;
  if (!out.sd_ok) ++stats_.sd_write_failures;
  if (sd_timeout) ++stats_.sd_write_timeouts;

  return out;
}

}  // namespace cores3sim
