#include "VirtualIna226.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace cores3sim {

bool VirtualIna226::validConversionTime(std::uint32_t us) {
  switch (us) {
    case 140:
    case 204:
    case 332:
    case 588:
    case 1100:
    case 2116:
    case 4156:
    case 8244:
      return true;
  }
  return false;
}

bool VirtualIna226::validAverages(std::uint32_t averages) {
  switch (averages) {
    case 1:
    case 4:
    case 16:
    case 64:
    case 128:
    case 256:
    case 512:
    case 1024:
      return true;
  }
  return false;
}

std::uint32_t VirtualIna226::conversionCycleMs(const Ina226Input& input) {
  const std::uint64_t total_us =
      static_cast<std::uint64_t>(input.shunt_conversion_us +
                                 input.bus_conversion_us) *
      std::max<std::uint32_t>(1, input.averages);
  return static_cast<std::uint32_t>(std::max<std::uint64_t>(
      1, (total_us + 999u) / 1000u));
}

float VirtualIna226::quantize(float value, float lsb) {
  if (!(lsb > 0.0f) || !std::isfinite(value)) return value;
  return std::round(value / lsb) * lsb;
}

void VirtualIna226::clearRuntime() {
  initialized_ = false;
  reinitializing_ = false;
  has_measurement_ = false;
  stale_latched_ = false;
  reinit_ready_at_ms_ = 0;
  next_conversion_ready_ms_ = 0;
  last_measurement_ms_ = 0;
}

void VirtualIna226::startReinit(std::uint32_t now_ms,
                                std::uint32_t delay_ms) {
  initialized_ = false;
  reinitializing_ = true;
  has_measurement_ = false;
  stale_latched_ = false;
  reinit_ready_at_ms_ = now_ms + delay_ms;
  next_conversion_ready_ms_ = 0;
  ++stats_.reinit_attempts;
}

Ina226Status VirtualIna226::update(std::uint32_t now_ms,
                                   const Ina226Input& input,
                                   bool bus_ok) {
  Ina226Status out;
  out.active = input.enabled;

  if (!input.enabled) {
    clearRuntime();
    reset_latched_ = false;
    ack_missing_latched_ = false;
    out.initialized = true;
    out.conversion_fresh = true;
    out.calibration_ok = true;
    out.configuration_ok = true;
    return out;
  }

  const bool config_ok =
      validConversionTime(input.shunt_conversion_us) &&
      validConversionTime(input.bus_conversion_us) &&
      validAverages(input.averages) &&
      input.shunt_ohm > 0.0f &&
      input.current_lsb_a > 0.0f;
  out.configuration_ok = config_ok;

  const bool reset_edge = input.force_reset && !reset_latched_;
  reset_latched_ = input.force_reset;

  const bool device_ok = bus_ok && input.powered && input.device_ack;
  out.device_ok = device_ok;

  const bool ack_missing = bus_ok && input.powered && !input.device_ack;
  if (ack_missing && !ack_missing_latched_) ++stats_.device_nacks;
  ack_missing_latched_ = ack_missing;

  if (!config_ok) {
    ++stats_.config_errors;
  }

  if (!input.powered || !input.device_ack) {
    clearRuntime();
  } else {
    if (reset_edge) {
      ++stats_.resets;
      clearRuntime();
    }

    if (reinitializing_ && (!bus_ok || !config_ok)) {
      reinitializing_ = false;
      reinit_ready_at_ms_ = 0;
    }

    if (!initialized_ &&
        !reinitializing_ &&
        input.auto_reinit &&
        bus_ok &&
        config_ok) {
      startReinit(now_ms, input.reinit_delay_ms);
    }

    if (reinitializing_ &&
        bus_ok &&
        config_ok &&
        now_ms >= reinit_ready_at_ms_) {
      reinitializing_ = false;
      initialized_ = true;
      has_measurement_ = false;
      next_conversion_ready_ms_ = now_ms + conversionCycleMs(input);
      ++stats_.reinit_successes;
    }

    if (initialized_ &&
        device_ok &&
        config_ok &&
        input.conversion_enabled &&
        !input.stall_conversions) {
      const std::uint32_t cycle_ms = conversionCycleMs(input);
      if (next_conversion_ready_ms_ == 0) {
        next_conversion_ready_ms_ = now_ms + cycle_ms;
      }

      if (now_ms >= next_conversion_ready_ms_) {
        const std::uint32_t cycles =
            1 + (now_ms - next_conversion_ready_ms_) / cycle_ms;
        next_conversion_ready_ms_ += cycles * cycle_ms;
        stats_.conversions_completed += cycles;

        const float shunt_v = input.current_a * input.shunt_ohm;
        const bool finite = std::isfinite(input.bus_voltage_v) &&
                            std::isfinite(input.current_a) &&
                            std::isfinite(shunt_v);
        const bool range_ok =
            finite &&
            input.bus_voltage_v >= 0.0f &&
            input.bus_voltage_v <= kMaxBusVoltageV &&
            shunt_v >= kMinShuntVoltageV &&
            shunt_v <= kMaxShuntVoltageV;

        last_range_ok_ = range_ok;
        last_math_overflow_ = input.force_math_overflow;

        if (!range_ok) {
          stats_.range_errors += cycles;
        } else {
          const float bus_v = quantize(input.bus_voltage_v, 0.00125f);
          const float shunt_q = quantize(shunt_v, 0.0000025f);

          last_bus_voltage_v_ = bus_v;
          last_shunt_voltage_v_ = shunt_q;

          if (input.calibration_programmed) {
            last_current_a_ = quantize(input.current_a, input.current_lsb_a);
            last_power_w_ = quantize(last_current_a_ * bus_v,
                                     input.current_lsb_a * 25.0f);
          } else {
            last_current_a_ = 0.0f;
            last_power_w_ = 0.0f;
            stats_.calibration_missing_conversions += cycles;
          }

          last_measurement_ms_ = now_ms;
          has_measurement_ = true;
        }

        if (input.force_math_overflow) {
          stats_.math_overflows += cycles;
        }
      }
    }
  }

  out.initialized = initialized_;
  out.reinitializing = reinitializing_;
  out.calibration_ok = input.calibration_programmed;
  out.range_ok = last_range_ok_;
  out.math_overflow = last_math_overflow_;
  out.bus_voltage_v = last_bus_voltage_v_;
  out.current_a = last_current_a_;
  out.power_w = last_power_w_;
  out.shunt_voltage_v = last_shunt_voltage_v_;

  if (has_measurement_) {
    out.measurement_age_ms = now_ms - last_measurement_ms_;
    const std::uint32_t stale_threshold =
        std::max<std::uint32_t>(100, conversionCycleMs(input) * 3);
    out.conversion_fresh =
        initialized_ && device_ok && out.measurement_age_ms <= stale_threshold;
  } else {
    out.measurement_age_ms = std::numeric_limits<std::uint32_t>::max();
    out.conversion_fresh = false;
  }

  const bool stale_now = initialized_ && !out.conversion_fresh;
  if (stale_now && !stale_latched_) ++stats_.stale_events;
  stale_latched_ = stale_now;

  return out;
}

}  // namespace cores3sim
