#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace cores3sim::replay {

#pragma pack(push, 1)
struct BoatLogHeaderRaw {
  char magic[8]{'B','O','A','T','L','O','G','1'};
  std::uint16_t version{1};
  std::uint16_t record_bytes{40};
  std::uint32_t boot_id{0};
  std::uint32_t start_millis{0};
};

struct BoatLogRecordRaw {
  std::uint32_t uptime_ms{0};
  std::int32_t latitude_e7{0};
  std::int32_t longitude_e7{0};
  std::int32_t vesc_erpm{0};
  std::int16_t speed_cm_per_s{0};
  std::int16_t roll_millirad{0};
  std::int16_t pitch_millirad{0};
  std::int16_t tof_mm{0};
  std::int16_t left_milli{0};
  std::int16_t right_milli{0};
  std::int16_t rear_milli{0};
  std::int16_t target_duty_ten_thousandth{0};
  std::int16_t applied_duty_ten_thousandth{0};
  std::uint16_t waypoint_distance_cm{0};
  std::uint8_t safety{0};
  std::uint8_t mode{0};
  std::uint8_t yaw_criteria{0};
  std::uint8_t flags{0};
};
#pragma pack(pop)

static_assert(sizeof(BoatLogHeaderRaw) == 20, "BOATLOG1 header ABI");
static_assert(sizeof(BoatLogRecordRaw) == 40, "BOATLOG1 record ABI");

struct BoatLogSample {
  std::uint32_t uptime_ms{0};
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double speed_mps{0.0};
  double roll_rad{0.0};
  double pitch_rad{0.0};
  double tof_m{0.0};
  double left{0.0};
  double right{0.0};
  double rear{0.0};
  double target_duty{0.0};
  double applied_duty{0.0};
  double waypoint_distance_m{0.0};
  double vesc_erpm{0.0};
  std::uint8_t safety{0};
  std::uint8_t mode{0};
  std::uint8_t yaw_criteria{0};
  std::uint8_t flags{0};
};

struct BoatLogFile {
  BoatLogHeaderRaw header{};
  std::vector<BoatLogSample> samples{};
  std::uint32_t trailing_bytes{0};
};

struct CompareThresholds {
  std::uint32_t max_time_delta_ms{150};
  double max_position_error_m{1.0};
  double max_speed_error_mps{0.15};
  double max_attitude_error_rad{0.02};
  double max_tof_error_m{0.03};
  double max_control_error{0.03};
  double max_duty_error{0.01};
  double max_waypoint_error_m{1.0};
  double max_erpm_error{50.0};
};

struct SampleComparison {
  std::size_t observed_index{0};
  std::size_t reference_index{0};
  std::uint32_t observed_ms{0};
  std::uint32_t reference_ms{0};
  std::uint32_t time_delta_ms{0};
  double position_error_m{0.0};
  double speed_error_mps{0.0};
  double roll_error_rad{0.0};
  double pitch_error_rad{0.0};
  double tof_error_m{0.0};
  double left_error{0.0};
  double right_error{0.0};
  double rear_error{0.0};
  double target_duty_error{0.0};
  double applied_duty_error{0.0};
  double waypoint_error_m{0.0};
  double erpm_error{0.0};
  bool safety_match{true};
  bool mode_match{true};
  bool yaw_criteria_match{true};
  bool flags_match{true};
  bool within_thresholds{true};
};

struct MetricSummary {
  double mean{0.0};
  double maximum{0.0};
};

struct ComparisonReport {
  bool pass{false};
  std::size_t observed_samples{0};
  std::size_t reference_samples{0};
  std::size_t matched_samples{0};
  std::size_t unmatched_observed{0};
  std::size_t threshold_failures{0};
  std::size_t safety_mismatches{0};
  std::size_t mode_mismatches{0};
  std::size_t yaw_criteria_mismatches{0};
  std::size_t flags_mismatches{0};
  MetricSummary time_delta_ms{};
  MetricSummary position_error_m{};
  MetricSummary speed_error_mps{};
  MetricSummary roll_error_rad{};
  MetricSummary pitch_error_rad{};
  MetricSummary tof_error_m{};
  MetricSummary left_error{};
  MetricSummary right_error{};
  MetricSummary rear_error{};
  MetricSummary target_duty_error{};
  MetricSummary applied_duty_error{};
  MetricSummary waypoint_error_m{};
  MetricSummary erpm_error{};
  std::vector<SampleComparison> samples{};
};

BoatLogSample decode(const BoatLogRecordRaw& raw);
BoatLogRecordRaw encode(const BoatLogSample& sample);

bool readBoatLog(const std::filesystem::path& path,
                 BoatLogFile& out,
                 std::string& error);
bool writeBoatLog(const std::filesystem::path& path,
                  const BoatLogHeaderRaw& header,
                  const std::vector<BoatLogSample>& samples,
                  std::string& error);
bool writeDecodedCsv(const std::filesystem::path& path,
                     const BoatLogFile& log,
                     std::string& error);

ComparisonReport compare(const BoatLogFile& observed,
                         const BoatLogFile& reference,
                         const CompareThresholds& thresholds = {});

bool writeComparisonCsv(const std::filesystem::path& path,
                        const ComparisonReport& report,
                        std::string& error);
bool writeSummaryJson(const std::filesystem::path& path,
                      const ComparisonReport& report,
                      const CompareThresholds& thresholds,
                      std::string& error);
bool writeSummarySvg(const std::filesystem::path& path,
                     const ComparisonReport& report,
                     const CompareThresholds& thresholds,
                     std::string& error);

}  // namespace cores3sim::replay
