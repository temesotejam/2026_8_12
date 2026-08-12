#include "BoatLogReplay.h"
#include "production_control.h"

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace cores3sim::replay;

static BoatLogSample makeSample(production_control::Controller& controller,
                                std::uint32_t t_ms,
                                std::size_t index) {
  const std::uint64_t now_us = static_cast<std::uint64_t>(t_ms) * 1000ULL;
  controller.setManual({0.20f, -0.20f, 0.10f, 0.30f,
                        production_control::ManualAll}, now_us);

  production_control::SensorInput input{};
  input.heartbeat = true;
  input.imuValid = true;
  input.tofValid = true;
  input.gnssValid = true;
  input.powerValid = true;
  input.vescValid = true;
  input.safety = production_control::AuthoritativeSafety::Running;
  input.nowUs = now_us;
  input.heartbeatUs = now_us;
  input.imuUs = now_us;
  input.tofUs = now_us;
  input.gnssUs = now_us;
  input.powerUs = now_us;
  input.vescUs = now_us;
  input.rollRad = 0.020f + static_cast<float>(index) * 0.001f;
  input.pitchRad = -0.010f + static_cast<float>(index) * 0.0005f;
  input.tofM = 0.45f + static_cast<float>(index) * 0.002f;
  input.groundSpeedMps = 2.50f + static_cast<float>(index) * 0.05f;
  input.busVoltageV = 12.2f;
  input.currentA = 2.0f;
  input.powerW = 24.4f;
  input.vescErpm = 1200.0f + static_cast<float>(index) * 20.0f;

  const auto output = controller.step(input);

  BoatLogSample sample{};
  sample.uptime_ms = t_ms;
  sample.latitude_deg = 36.4000000 + static_cast<double>(index) * 0.0000100;
  sample.longitude_deg = 136.6000000 + static_cast<double>(index) * 0.0000120;
  sample.speed_mps = input.groundSpeedMps;
  sample.roll_rad = input.rollRad;
  sample.pitch_rad = input.pitchRad;
  sample.tof_m = input.tofM;
  sample.left = output.leftFront;
  sample.right = output.rightFront;
  sample.rear = output.rearYaw;
  sample.target_duty = output.propulsion * 0.60;
  sample.applied_duty = sample.target_duty;
  sample.waypoint_distance_m = 0.0;
  sample.vesc_erpm = input.vescErpm;
  sample.safety = static_cast<std::uint8_t>(input.safety);
  sample.mode = static_cast<std::uint8_t>(controller.mode());
  sample.yaw_criteria = 0;
  // Matches BOATLOG1 v1 flags: GNSS + IMU + ToF + snapshot + link.
  sample.flags = 1u | 2u | 4u | 16u | 32u;
  return sample;
}

int main(int argc, char** argv) {
  const fs::path outdir = argc > 1 ? argv[1] : "artifacts/phase10_fixture";
  fs::create_directories(outdir);

  production_control::Controller controller;
  const auto mode_result = controller.setMode(
      production_control::ControlMode::Manual, 1001,
      production_control::AuthoritativeSafety::Disarmed);
  if (mode_result.ack != production_control::Ack::Accepted) {
    std::cerr << "failed to configure production controller\n";
    return 2;
  }

  std::vector<BoatLogSample> reference;
  for (std::size_t i = 0; i < 6; ++i) {
    reference.push_back(makeSample(controller, 2000u + static_cast<std::uint32_t>(i) * 2000u, i));
  }

  auto shifted = reference;
  for (auto& sample : shifted) sample.uptime_ms += 80;

  auto perturbed = reference;
  if (perturbed.size() >= 4) {
    perturbed[2].latitude_deg += 0.0000500;  // ~5.6 m north.
    perturbed[2].left += 0.08;
    perturbed[2].applied_duty += 0.03;
    perturbed[3].safety = 1;
    perturbed[4].vesc_erpm += 200.0;
  }

  BoatLogHeaderRaw header{};
  header.boot_id = 0x50483130u;  // "PH10"
  header.start_millis = 1234;

  std::string error;
  if (!writeBoatLog(outdir / "reference.bin", header, reference, error) ||
      !writeBoatLog(outdir / "observed_exact.bin", header, reference, error) ||
      !writeBoatLog(outdir / "observed_shifted_80ms.bin", header, shifted, error) ||
      !writeBoatLog(outdir / "observed_perturbed.bin", header, perturbed, error)) {
    std::cerr << error << '\n';
    return 2;
  }

  BoatLogFile decoded{};
  if (!readBoatLog(outdir / "reference.bin", decoded, error) ||
      !writeDecodedCsv(outdir / "reference_decoded.csv", decoded, error)) {
    std::cerr << error << '\n';
    return 2;
  }

  std::cout << "Generated BOATLOG1 fixtures: " << reference.size()
            << " records, cadence=2000ms\n";
  return 0;
}
