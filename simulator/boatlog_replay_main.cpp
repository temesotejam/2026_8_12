#include "BoatLogReplay.h"

#include <filesystem>
#include <iostream>
#include <string>

namespace fs = std::filesystem;
using namespace cores3sim::replay;

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "usage: boatlog_replay <observed.bin> <reference.bin> <outdir> [--allow-fail]\n";
    return 2;
  }

  const fs::path observed_path = argv[1];
  const fs::path reference_path = argv[2];
  const fs::path outdir = argv[3];
  const bool allow_fail = argc >= 5 && std::string(argv[4]) == "--allow-fail";
  fs::create_directories(outdir);

  BoatLogFile observed{}, reference{};
  std::string error;
  if (!readBoatLog(observed_path, observed, error)) {
    std::cerr << "observed: " << error << '\n';
    return 2;
  }
  if (!readBoatLog(reference_path, reference, error)) {
    std::cerr << "reference: " << error << '\n';
    return 2;
  }

  CompareThresholds thresholds{};
  const auto report = compare(observed, reference, thresholds);

  if (!writeDecodedCsv(outdir / "observed_decoded.csv", observed, error) ||
      !writeDecodedCsv(outdir / "reference_decoded.csv", reference, error) ||
      !writeComparisonCsv(outdir / "comparison.csv", report, error) ||
      !writeSummaryJson(outdir / "summary.json", report, thresholds, error) ||
      !writeSummarySvg(outdir / "summary.svg", report, thresholds, error)) {
    std::cerr << error << '\n';
    return 2;
  }

  std::cout << "BOATLOG1 observed=" << report.observed_samples
            << " reference=" << report.reference_samples
            << " matched=" << report.matched_samples
            << " failures=" << report.threshold_failures
            << " result=" << (report.pass ? "PASS" : "FAIL") << '\n';
  std::cout << "max position=" << report.position_error_m.maximum
            << "m control(L/R/R)=" << report.left_error.maximum << '/'
            << report.right_error.maximum << '/' << report.rear_error.maximum
            << " duty=" << report.applied_duty_error.maximum
            << " erpm=" << report.erpm_error.maximum << '\n';

  if (!report.pass && !allow_fail) return 1;
  return 0;
}
