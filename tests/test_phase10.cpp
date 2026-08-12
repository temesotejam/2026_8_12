#include "BoatLogReplay.h"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace cores3sim::replay;
namespace fs = std::filesystem;

namespace {
void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

BoatLogSample sample(std::uint32_t ms, double offset = 0.0) {
  BoatLogSample s{};
  s.uptime_ms = ms;
  s.latitude_deg = 36.4 + offset * 0.00001;
  s.longitude_deg = 136.6 + offset * 0.00001;
  s.speed_mps = 2.5 + offset * 0.05;
  s.roll_rad = 0.02 + offset * 0.001;
  s.pitch_rad = -0.01 + offset * 0.001;
  s.tof_m = 0.45 + offset * 0.002;
  s.left = 0.20;
  s.right = -0.20;
  s.rear = 0.10;
  s.target_duty = 0.18;
  s.applied_duty = 0.18;
  s.waypoint_distance_m = 12.3 + offset;
  s.vesc_erpm = 1200 + offset * 10;
  s.safety = 3;
  s.mode = 0;
  s.yaw_criteria = 0;
  s.flags = 55;
  return s;
}
}

int main() {
  static_assert(sizeof(BoatLogHeaderRaw) == 20, "BOATLOG1 header size");
  static_assert(sizeof(BoatLogRecordRaw) == 40, "BOATLOG1 record size");

  const fs::path root = fs::temp_directory_path() / "cores3_phase10_boatlog";
  fs::remove_all(root);
  fs::create_directories(root);

  BoatLogHeaderRaw header{};
  header.boot_id = 0x12345678;
  header.start_millis = 345;
  std::vector<BoatLogSample> reference{
      sample(2000, 0), sample(4000, 1), sample(6000, 2)};
  std::string error;
  require(writeBoatLog(root / "reference.bin", header, reference, error),
          "write reference BOATLOG1");

  BoatLogFile decoded{};
  require(readBoatLog(root / "reference.bin", decoded, error),
          "read reference BOATLOG1");
  require(decoded.samples.size() == 3 && decoded.trailing_bytes == 0,
          "record count and no trailing bytes");
  require(decoded.header.boot_id == header.boot_id &&
              decoded.header.start_millis == header.start_millis,
          "header metadata round trip");
  require(std::fabs(decoded.samples[1].speed_mps - 2.55) < 0.011 &&
              std::fabs(decoded.samples[1].roll_rad - 0.021) < 0.0011,
          "quantized sample decode");

  const auto exact = compare(decoded, decoded);
  require(exact.pass && exact.matched_samples == 3 &&
              exact.threshold_failures == 0,
          "exact BOATLOG1 comparison passes");

  auto shifted_samples = reference;
  for (auto& s : shifted_samples) s.uptime_ms += 80;
  require(writeBoatLog(root / "shifted.bin", header, shifted_samples, error),
          "write shifted BOATLOG1");
  BoatLogFile shifted{};
  require(readBoatLog(root / "shifted.bin", shifted, error),
          "read shifted BOATLOG1");
  const auto shifted_report = compare(shifted, decoded);
  require(shifted_report.pass && shifted_report.time_delta_ms.maximum == 80,
          "nearest-time alignment accepts 80ms shift");

  auto perturbed_samples = reference;
  perturbed_samples[1].latitude_deg += 0.00005;
  perturbed_samples[1].left += 0.08;
  perturbed_samples[1].applied_duty += 0.03;
  perturbed_samples[2].safety = 1;
  require(writeBoatLog(root / "perturbed.bin", header, perturbed_samples, error),
          "write perturbed BOATLOG1");
  BoatLogFile perturbed{};
  require(readBoatLog(root / "perturbed.bin", perturbed, error),
          "read perturbed BOATLOG1");
  const auto bad = compare(perturbed, decoded);
  require(!bad.pass && bad.threshold_failures >= 2 && bad.safety_mismatches == 1,
          "comparator detects numeric and safety mismatches");
  require(bad.position_error_m.maximum > 5.0 && bad.left_error.maximum > 0.07,
          "mismatch magnitudes are reported");

  {
    std::ofstream append(root / "reference.bin", std::ios::binary | std::ios::app);
    const char tail[3] = {1, 2, 3};
    append.write(tail, sizeof(tail));
  }
  BoatLogFile trailing{};
  require(readBoatLog(root / "reference.bin", trailing, error) &&
              trailing.samples.size() == 3 && trailing.trailing_bytes == 3,
          "power-loss-style trailing partial record is reported but prior records survive");

  BoatLogHeaderRaw bad_magic = header;
  bad_magic.magic[0] = 'X';
  require(writeBoatLog(root / "bad_magic.bin", bad_magic, reference, error),
          "write bad magic fixture");
  BoatLogFile invalid{};
  require(!readBoatLog(root / "bad_magic.bin", invalid, error) &&
              error.find("magic") != std::string::npos,
          "bad magic rejected");

  BoatLogHeaderRaw bad_version = header;
  bad_version.version = 2;
  require(writeBoatLog(root / "bad_version.bin", bad_version, reference, error),
          "write bad version fixture");
  require(!readBoatLog(root / "bad_version.bin", invalid, error) &&
              error.find("version") != std::string::npos,
          "unknown version rejected");

  BoatLogHeaderRaw bad_size = header;
  bad_size.record_bytes = 41;
  require(writeBoatLog(root / "bad_size.bin", bad_size, reference, error),
          "write bad record-size fixture");
  require(!readBoatLog(root / "bad_size.bin", invalid, error) &&
              error.find("record size") != std::string::npos,
          "unknown record size rejected");

  require(writeDecodedCsv(root / "decoded.csv", decoded, error),
          "write decoded CSV");
  require(writeComparisonCsv(root / "comparison.csv", bad, error),
          "write comparison CSV");
  CompareThresholds thresholds{};
  require(writeSummaryJson(root / "summary.json", bad, thresholds, error),
          "write summary JSON");
  require(writeSummarySvg(root / "summary.svg", bad, thresholds, error),
          "write summary SVG");

  fs::remove_all(root);
  std::cout << "All phase-10 BOATLOG1 replay tests passed.\n";
  return 0;
}
