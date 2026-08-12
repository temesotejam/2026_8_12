#include "BoatLogReplay.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

namespace cores3sim::replay {
namespace {
constexpr double kEarthMPerDeg = 111320.0;
constexpr double kPi = 3.14159265358979323846;

template <typename T>
T clampRounded(double value) {
  if (!std::isfinite(value)) return 0;
  const double low = static_cast<double>(std::numeric_limits<T>::min());
  const double high = static_cast<double>(std::numeric_limits<T>::max());
  if (value <= low) return std::numeric_limits<T>::min();
  if (value >= high) return std::numeric_limits<T>::max();
  return static_cast<T>(std::llround(value));
}

std::uint16_t clampU16(double value) {
  if (!std::isfinite(value) || value <= 0.0) return 0;
  if (value >= 65535.0) return 65535;
  return static_cast<std::uint16_t>(std::llround(value));
}

std::uint32_t absDelta(std::uint32_t a, std::uint32_t b) {
  return a >= b ? a - b : b - a;
}

double positionErrorM(const BoatLogSample& a, const BoatLogSample& b) {
  const double mean_lat = (a.latitude_deg + b.latitude_deg) * 0.5 * kPi / 180.0;
  const double north = (a.latitude_deg - b.latitude_deg) * kEarthMPerDeg;
  const double east = (a.longitude_deg - b.longitude_deg) *
                      kEarthMPerDeg * std::cos(mean_lat);
  return std::sqrt(north * north + east * east);
}

void addMetric(MetricSummary& metric, double value, std::size_t count_before) {
  if (value > metric.maximum) metric.maximum = value;
  metric.mean = (metric.mean * static_cast<double>(count_before) + value) /
                static_cast<double>(count_before + 1);
}

const char* boolJson(bool value) { return value ? "true" : "false"; }

bool sampleWithin(const SampleComparison& c, const CompareThresholds& t) {
  return c.time_delta_ms <= t.max_time_delta_ms &&
         c.position_error_m <= t.max_position_error_m &&
         c.speed_error_mps <= t.max_speed_error_mps &&
         c.roll_error_rad <= t.max_attitude_error_rad &&
         c.pitch_error_rad <= t.max_attitude_error_rad &&
         c.tof_error_m <= t.max_tof_error_m &&
         c.left_error <= t.max_control_error &&
         c.right_error <= t.max_control_error &&
         c.rear_error <= t.max_control_error &&
         c.target_duty_error <= t.max_duty_error &&
         c.applied_duty_error <= t.max_duty_error &&
         c.waypoint_error_m <= t.max_waypoint_error_m &&
         c.erpm_error <= t.max_erpm_error &&
         c.safety_match && c.mode_match && c.yaw_criteria_match && c.flags_match;
}

std::string metricJson(const MetricSummary& metric) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(6)
      << "{\"mean\":" << metric.mean << ",\"max\":" << metric.maximum << "}";
  return out.str();
}
}

BoatLogSample decode(const BoatLogRecordRaw& raw) {
  BoatLogSample s{};
  s.uptime_ms = raw.uptime_ms;
  s.latitude_deg = static_cast<double>(raw.latitude_e7) / 1.0e7;
  s.longitude_deg = static_cast<double>(raw.longitude_e7) / 1.0e7;
  s.vesc_erpm = static_cast<double>(raw.vesc_erpm);
  s.speed_mps = static_cast<double>(raw.speed_cm_per_s) / 100.0;
  s.roll_rad = static_cast<double>(raw.roll_millirad) / 1000.0;
  s.pitch_rad = static_cast<double>(raw.pitch_millirad) / 1000.0;
  s.tof_m = static_cast<double>(raw.tof_mm) / 1000.0;
  s.left = static_cast<double>(raw.left_milli) / 1000.0;
  s.right = static_cast<double>(raw.right_milli) / 1000.0;
  s.rear = static_cast<double>(raw.rear_milli) / 1000.0;
  s.target_duty = static_cast<double>(raw.target_duty_ten_thousandth) / 10000.0;
  s.applied_duty = static_cast<double>(raw.applied_duty_ten_thousandth) / 10000.0;
  s.waypoint_distance_m = static_cast<double>(raw.waypoint_distance_cm) / 100.0;
  s.safety = raw.safety;
  s.mode = raw.mode;
  s.yaw_criteria = raw.yaw_criteria;
  s.flags = raw.flags;
  return s;
}

BoatLogRecordRaw encode(const BoatLogSample& s) {
  BoatLogRecordRaw raw{};
  raw.uptime_ms = s.uptime_ms;
  raw.latitude_e7 = clampRounded<std::int32_t>(s.latitude_deg * 1.0e7);
  raw.longitude_e7 = clampRounded<std::int32_t>(s.longitude_deg * 1.0e7);
  raw.vesc_erpm = clampRounded<std::int32_t>(s.vesc_erpm);
  raw.speed_cm_per_s = clampRounded<std::int16_t>(s.speed_mps * 100.0);
  raw.roll_millirad = clampRounded<std::int16_t>(s.roll_rad * 1000.0);
  raw.pitch_millirad = clampRounded<std::int16_t>(s.pitch_rad * 1000.0);
  raw.tof_mm = clampRounded<std::int16_t>(s.tof_m * 1000.0);
  raw.left_milli = clampRounded<std::int16_t>(s.left * 1000.0);
  raw.right_milli = clampRounded<std::int16_t>(s.right * 1000.0);
  raw.rear_milli = clampRounded<std::int16_t>(s.rear * 1000.0);
  raw.target_duty_ten_thousandth = clampRounded<std::int16_t>(s.target_duty * 10000.0);
  raw.applied_duty_ten_thousandth = clampRounded<std::int16_t>(s.applied_duty * 10000.0);
  raw.waypoint_distance_cm = clampU16(s.waypoint_distance_m * 100.0);
  raw.safety = s.safety;
  raw.mode = s.mode;
  raw.yaw_criteria = s.yaw_criteria;
  raw.flags = s.flags;
  return raw;
}

bool readBoatLog(const std::filesystem::path& path,
                 BoatLogFile& out,
                 std::string& error) {
  out = {};
  error.clear();
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file) {
    error = "cannot open BOATLOG1 file: " + path.string();
    return false;
  }
  const auto size = file.tellg();
  if (size < static_cast<std::streamoff>(sizeof(BoatLogHeaderRaw))) {
    error = "BOATLOG1 file is shorter than 20-byte header";
    return false;
  }
  file.seekg(0);
  file.read(reinterpret_cast<char*>(&out.header), sizeof(out.header));
  if (!file) {
    error = "failed to read BOATLOG1 header";
    return false;
  }
  const std::array<char, 8> expected{{'B','O','A','T','L','O','G','1'}};
  if (std::memcmp(out.header.magic, expected.data(), expected.size()) != 0) {
    error = "BOATLOG1 magic mismatch";
    return false;
  }
  if (out.header.version != 1) {
    error = "unsupported BOATLOG1 version: " + std::to_string(out.header.version);
    return false;
  }
  if (out.header.record_bytes != sizeof(BoatLogRecordRaw)) {
    error = "unexpected BOATLOG1 record size: " + std::to_string(out.header.record_bytes);
    return false;
  }

  const std::uint64_t payload = static_cast<std::uint64_t>(size) - sizeof(BoatLogHeaderRaw);
  const std::uint64_t records = payload / sizeof(BoatLogRecordRaw);
  out.trailing_bytes = static_cast<std::uint32_t>(payload % sizeof(BoatLogRecordRaw));
  out.samples.reserve(static_cast<std::size_t>(records));
  for (std::uint64_t i = 0; i < records; ++i) {
    BoatLogRecordRaw raw{};
    file.read(reinterpret_cast<char*>(&raw), sizeof(raw));
    if (!file) {
      error = "failed while reading BOATLOG1 record " + std::to_string(i);
      return false;
    }
    out.samples.push_back(decode(raw));
  }
  return true;
}

bool writeBoatLog(const std::filesystem::path& path,
                  const BoatLogHeaderRaw& header,
                  const std::vector<BoatLogSample>& samples,
                  std::string& error) {
  error.clear();
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  if (!file) {
    error = "cannot create BOATLOG1 file: " + path.string();
    return false;
  }
  file.write(reinterpret_cast<const char*>(&header), sizeof(header));
  for (const auto& sample : samples) {
    const auto raw = encode(sample);
    file.write(reinterpret_cast<const char*>(&raw), sizeof(raw));
  }
  if (!file) {
    error = "failed writing BOATLOG1 file: " + path.string();
    return false;
  }
  return true;
}

bool writeDecodedCsv(const std::filesystem::path& path,
                     const BoatLogFile& log,
                     std::string& error) {
  error.clear();
  std::ofstream out(path);
  if (!out) {
    error = "cannot create decoded CSV: " + path.string();
    return false;
  }
  out << "uptime_ms,latitude_deg,longitude_deg,speed_mps,roll_rad,pitch_rad,tof_m,left,right,rear,target_duty,applied_duty,waypoint_distance_m,vesc_erpm,safety,mode,yaw_criteria,flags\n";
  out << std::fixed << std::setprecision(7);
  for (const auto& s : log.samples) {
    out << s.uptime_ms << ',' << s.latitude_deg << ',' << s.longitude_deg << ','
        << s.speed_mps << ',' << s.roll_rad << ',' << s.pitch_rad << ','
        << s.tof_m << ',' << s.left << ',' << s.right << ',' << s.rear << ','
        << s.target_duty << ',' << s.applied_duty << ',' << s.waypoint_distance_m
        << ',' << s.vesc_erpm << ',' << static_cast<unsigned>(s.safety) << ','
        << static_cast<unsigned>(s.mode) << ',' << static_cast<unsigned>(s.yaw_criteria)
        << ',' << static_cast<unsigned>(s.flags) << '\n';
  }
  return true;
}

ComparisonReport compare(const BoatLogFile& observed,
                         const BoatLogFile& reference,
                         const CompareThresholds& thresholds) {
  ComparisonReport report{};
  report.observed_samples = observed.samples.size();
  report.reference_samples = reference.samples.size();
  if (reference.samples.empty()) {
    report.unmatched_observed = observed.samples.size();
    return report;
  }

  std::size_t aggregate_count = 0;
  for (std::size_t oi = 0; oi < observed.samples.size(); ++oi) {
    const auto& obs = observed.samples[oi];
    auto lower = std::lower_bound(
        reference.samples.begin(), reference.samples.end(), obs.uptime_ms,
        [](const BoatLogSample& sample, std::uint32_t ms) {
          return sample.uptime_ms < ms;
        });
    std::size_t ri = 0;
    if (lower == reference.samples.begin()) {
      ri = 0;
    } else if (lower == reference.samples.end()) {
      ri = reference.samples.size() - 1;
    } else {
      const std::size_t high = static_cast<std::size_t>(lower - reference.samples.begin());
      const std::size_t low = high - 1;
      ri = absDelta(obs.uptime_ms, reference.samples[low].uptime_ms) <=
                   absDelta(obs.uptime_ms, reference.samples[high].uptime_ms)
               ? low : high;
    }
    const auto& ref = reference.samples[ri];
    SampleComparison c{};
    c.observed_index = oi;
    c.reference_index = ri;
    c.observed_ms = obs.uptime_ms;
    c.reference_ms = ref.uptime_ms;
    c.time_delta_ms = absDelta(obs.uptime_ms, ref.uptime_ms);
    if (c.time_delta_ms > thresholds.max_time_delta_ms) {
      ++report.unmatched_observed;
      continue;
    }
    c.position_error_m = positionErrorM(obs, ref);
    c.speed_error_mps = std::abs(obs.speed_mps - ref.speed_mps);
    c.roll_error_rad = std::abs(obs.roll_rad - ref.roll_rad);
    c.pitch_error_rad = std::abs(obs.pitch_rad - ref.pitch_rad);
    c.tof_error_m = std::abs(obs.tof_m - ref.tof_m);
    c.left_error = std::abs(obs.left - ref.left);
    c.right_error = std::abs(obs.right - ref.right);
    c.rear_error = std::abs(obs.rear - ref.rear);
    c.target_duty_error = std::abs(obs.target_duty - ref.target_duty);
    c.applied_duty_error = std::abs(obs.applied_duty - ref.applied_duty);
    c.waypoint_error_m = std::abs(obs.waypoint_distance_m - ref.waypoint_distance_m);
    c.erpm_error = std::abs(obs.vesc_erpm - ref.vesc_erpm);
    c.safety_match = obs.safety == ref.safety;
    c.mode_match = obs.mode == ref.mode;
    c.yaw_criteria_match = obs.yaw_criteria == ref.yaw_criteria;
    c.flags_match = obs.flags == ref.flags;
    c.within_thresholds = sampleWithin(c, thresholds);

    addMetric(report.time_delta_ms, static_cast<double>(c.time_delta_ms), aggregate_count);
    addMetric(report.position_error_m, c.position_error_m, aggregate_count);
    addMetric(report.speed_error_mps, c.speed_error_mps, aggregate_count);
    addMetric(report.roll_error_rad, c.roll_error_rad, aggregate_count);
    addMetric(report.pitch_error_rad, c.pitch_error_rad, aggregate_count);
    addMetric(report.tof_error_m, c.tof_error_m, aggregate_count);
    addMetric(report.left_error, c.left_error, aggregate_count);
    addMetric(report.right_error, c.right_error, aggregate_count);
    addMetric(report.rear_error, c.rear_error, aggregate_count);
    addMetric(report.target_duty_error, c.target_duty_error, aggregate_count);
    addMetric(report.applied_duty_error, c.applied_duty_error, aggregate_count);
    addMetric(report.waypoint_error_m, c.waypoint_error_m, aggregate_count);
    addMetric(report.erpm_error, c.erpm_error, aggregate_count);
    ++aggregate_count;
    ++report.matched_samples;
    if (!c.within_thresholds) ++report.threshold_failures;
    if (!c.safety_match) ++report.safety_mismatches;
    if (!c.mode_match) ++report.mode_mismatches;
    if (!c.yaw_criteria_match) ++report.yaw_criteria_mismatches;
    if (!c.flags_match) ++report.flags_mismatches;
    report.samples.push_back(c);
  }

  report.pass = report.observed_samples > 0 &&
                report.matched_samples == report.observed_samples &&
                report.unmatched_observed == 0 &&
                report.threshold_failures == 0;
  return report;
}

bool writeComparisonCsv(const std::filesystem::path& path,
                        const ComparisonReport& report,
                        std::string& error) {
  error.clear();
  std::ofstream out(path);
  if (!out) {
    error = "cannot create comparison CSV: " + path.string();
    return false;
  }
  out << "observed_index,reference_index,observed_ms,reference_ms,time_delta_ms,position_error_m,speed_error_mps,roll_error_rad,pitch_error_rad,tof_error_m,left_error,right_error,rear_error,target_duty_error,applied_duty_error,waypoint_error_m,erpm_error,safety_match,mode_match,yaw_criteria_match,flags_match,within_thresholds\n";
  out << std::fixed << std::setprecision(6);
  for (const auto& c : report.samples) {
    out << c.observed_index << ',' << c.reference_index << ',' << c.observed_ms << ','
        << c.reference_ms << ',' << c.time_delta_ms << ',' << c.position_error_m << ','
        << c.speed_error_mps << ',' << c.roll_error_rad << ',' << c.pitch_error_rad
        << ',' << c.tof_error_m << ',' << c.left_error << ',' << c.right_error << ','
        << c.rear_error << ',' << c.target_duty_error << ',' << c.applied_duty_error
        << ',' << c.waypoint_error_m << ',' << c.erpm_error << ',' << c.safety_match
        << ',' << c.mode_match << ',' << c.yaw_criteria_match << ',' << c.flags_match
        << ',' << c.within_thresholds << '\n';
  }
  return true;
}

bool writeSummaryJson(const std::filesystem::path& path,
                      const ComparisonReport& r,
                      const CompareThresholds& t,
                      std::string& error) {
  error.clear();
  std::ofstream out(path);
  if (!out) {
    error = "cannot create summary JSON: " + path.string();
    return false;
  }
  out << std::fixed << std::setprecision(6)
      << "{\n  \"pass\": " << boolJson(r.pass)
      << ",\n  \"observed_samples\": " << r.observed_samples
      << ",\n  \"reference_samples\": " << r.reference_samples
      << ",\n  \"matched_samples\": " << r.matched_samples
      << ",\n  \"unmatched_observed\": " << r.unmatched_observed
      << ",\n  \"threshold_failures\": " << r.threshold_failures
      << ",\n  \"categorical_mismatches\": {\"safety\": " << r.safety_mismatches
      << ", \"mode\": " << r.mode_mismatches
      << ", \"yaw_criteria\": " << r.yaw_criteria_mismatches
      << ", \"flags\": " << r.flags_mismatches << "},\n"
      << "  \"metrics\": {\n"
      << "    \"time_delta_ms\": " << metricJson(r.time_delta_ms) << ",\n"
      << "    \"position_error_m\": " << metricJson(r.position_error_m) << ",\n"
      << "    \"speed_error_mps\": " << metricJson(r.speed_error_mps) << ",\n"
      << "    \"roll_error_rad\": " << metricJson(r.roll_error_rad) << ",\n"
      << "    \"pitch_error_rad\": " << metricJson(r.pitch_error_rad) << ",\n"
      << "    \"tof_error_m\": " << metricJson(r.tof_error_m) << ",\n"
      << "    \"left_error\": " << metricJson(r.left_error) << ",\n"
      << "    \"right_error\": " << metricJson(r.right_error) << ",\n"
      << "    \"rear_error\": " << metricJson(r.rear_error) << ",\n"
      << "    \"target_duty_error\": " << metricJson(r.target_duty_error) << ",\n"
      << "    \"applied_duty_error\": " << metricJson(r.applied_duty_error) << ",\n"
      << "    \"waypoint_error_m\": " << metricJson(r.waypoint_error_m) << ",\n"
      << "    \"erpm_error\": " << metricJson(r.erpm_error) << "\n  },\n"
      << "  \"thresholds\": {\"time_ms\": " << t.max_time_delta_ms
      << ", \"position_m\": " << t.max_position_error_m
      << ", \"speed_mps\": " << t.max_speed_error_mps
      << ", \"attitude_rad\": " << t.max_attitude_error_rad
      << ", \"tof_m\": " << t.max_tof_error_m
      << ", \"control\": " << t.max_control_error
      << ", \"duty\": " << t.max_duty_error
      << ", \"waypoint_m\": " << t.max_waypoint_error_m
      << ", \"erpm\": " << t.max_erpm_error << "}\n}\n";
  return true;
}

bool writeSummarySvg(const std::filesystem::path& path,
                     const ComparisonReport& r,
                     const CompareThresholds&,
                     std::string& error) {
  error.clear();
  std::ofstream out(path);
  if (!out) {
    error = "cannot create summary SVG: " + path.string();
    return false;
  }
  out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"900\" height=\"430\">"
      << "<rect width=\"900\" height=\"430\" fill=\"#101418\"/>"
      << "<g fill=\"white\" font-family=\"monospace\">"
      << "<text x=\"24\" y=\"34\" font-size=\"21\">Phase 10 - BOATLOG1 hardware vs host-simulation comparison</text>"
      << "<text x=\"24\" y=\"72\" font-size=\"24\">RESULT: " << (r.pass ? "PASS" : "FAIL") << "</text>"
      << "<text x=\"24\" y=\"108\" font-size=\"14\">matched=" << r.matched_samples
      << "/" << r.observed_samples << " threshold_failures=" << r.threshold_failures
      << " unmatched=" << r.unmatched_observed << "</text>"
      << "<text x=\"24\" y=\"138\" font-size=\"14\">position max=" << std::fixed << std::setprecision(3)
      << r.position_error_m.maximum << " m  speed max=" << r.speed_error_mps.maximum
      << " m/s</text>"
      << "<text x=\"24\" y=\"168\" font-size=\"14\">roll/pitch max=" << r.roll_error_rad.maximum
      << "/" << r.pitch_error_rad.maximum << " rad  ToF max=" << r.tof_error_m.maximum << " m</text>"
      << "<text x=\"24\" y=\"198\" font-size=\"14\">left/right/rear max=" << r.left_error.maximum
      << "/" << r.right_error.maximum << "/" << r.rear_error.maximum << "</text>"
      << "<text x=\"24\" y=\"228\" font-size=\"14\">target/applied duty max=" << r.target_duty_error.maximum
      << "/" << r.applied_duty_error.maximum << "  ERPM max=" << r.erpm_error.maximum << "</text>"
      << "<text x=\"24\" y=\"258\" font-size=\"14\">safety/mode/yaw/flags mismatches="
      << r.safety_mismatches << "/" << r.mode_mismatches << "/"
      << r.yaw_criteria_mismatches << "/" << r.flags_mismatches << "</text>"
      << "<text x=\"24\" y=\"310\" font-size=\"13\">BOATLOG1 v1: 20-byte header + 40-byte fixed little-endian records</text>"
      << "<text x=\"24\" y=\"338\" font-size=\"13\">This compares logged observables; it is not a full FreeRTOS/electrical replay.</text>"
      << "</g></svg>";
  return true;
}

}  // namespace cores3sim::replay
