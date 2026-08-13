#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "BoatLog2Format.h"
#include "production_control.h"

namespace cores3sim::replay2 {

struct BoatLog2File {
  BoatLog2HeaderRaw header{};
  std::vector<BoatLog2RecordRaw> records{};
  std::uint32_t trailing_bytes{0};
};

struct ReplayRow {
  std::uint32_t sequence{0};
  RecordType type{RecordType::Reset};
  bool match{true};
  float max_float_error{0.0f};
  std::string failure{};
};

struct ReplayReport {
  bool pass{false};
  std::size_t total_records{0};
  std::size_t processed_records{0};
  std::uint32_t trailing_bytes{0};
  std::size_t crc_failures{0};
  std::size_t sequence_failures{0};
  std::size_t malformed_records{0};
  std::size_t unknown_records{0};
  std::size_t command_mismatches{0};
  std::size_t step_mismatches{0};
  float max_float_error{0.0f};
  std::uint32_t first_failure_sequence{0};
  std::string first_failure_field{};
  std::vector<ReplayRow> rows{};
};

ConfigWire encodeConfig(const production_control::Config& config);
production_control::Config decodeConfig(const ConfigWire& wire);
SensorInputWire encodeSensorInput(const production_control::SensorInput& input);
production_control::SensorInput decodeSensorInput(const SensorInputWire& wire);
OutputWire encodeOutput(const production_control::Output& output);
production_control::Output decodeOutput(const OutputWire& wire);

std::uint32_t recordCrc(const BoatLog2RecordRaw& record);

BoatLog2RecordRaw makeResetRecord(std::uint32_t sequence, std::uint64_t at_us);
BoatLog2RecordRaw makeConfigRecord(std::uint32_t sequence, std::uint64_t at_us,
                                   const production_control::Config& config);
BoatLog2RecordRaw makeModeRecord(std::uint32_t sequence, std::uint64_t at_us,
                                 production_control::ControlMode mode,
                                 std::uint32_t request_id,
                                 production_control::AuthoritativeSafety safety,
                                 production_control::CommandResult expected);
BoatLog2RecordRaw makeManualRecord(std::uint32_t sequence, std::uint64_t at_us,
                                   const production_control::ManualCommand& command,
                                   std::uint64_t received_us,
                                   production_control::CommandResult expected);
BoatLog2RecordRaw makeHeadingRecord(std::uint32_t sequence, std::uint64_t at_us,
                                    float yaw_rad, std::uint32_t request_id,
                                    production_control::CommandResult expected);
BoatLog2RecordRaw makeWaypointsRecord(std::uint32_t sequence, std::uint64_t at_us,
                                      const production_control::Waypoint* points,
                                      std::uint8_t count,
                                      std::uint32_t request_id,
                                      production_control::AuthoritativeSafety safety,
                                      production_control::CommandResult expected);
BoatLog2RecordRaw makeReachRecord(std::uint32_t sequence, std::uint64_t at_us,
                                  float radius_m,
                                  production_control::AuthoritativeSafety safety,
                                  production_control::CommandResult expected);
BoatLog2RecordRaw makeStepRecord(std::uint32_t sequence, std::uint64_t at_us,
                                 const production_control::SensorInput& input,
                                 const production_control::Output& expected);

bool readBoatLog2(const std::filesystem::path& path, BoatLog2File& out,
                  std::string& error);
bool writeBoatLog2(const std::filesystem::path& path,
                   const BoatLog2HeaderRaw& header,
                   const std::vector<BoatLog2RecordRaw>& records,
                   std::string& error);

ReplayReport replay(const BoatLog2File& log,
                    float float_epsilon = kDefaultFloatEpsilon);

bool writeReplayCsv(const std::filesystem::path& path,
                    const ReplayReport& report,
                    std::string& error);
bool writeReplayJson(const std::filesystem::path& path,
                     const ReplayReport& report,
                     float float_epsilon,
                     std::string& error);
bool writeReplaySvg(const std::filesystem::path& path,
                    const ReplayReport& report,
                    float float_epsilon,
                    std::string& error);

}  // namespace cores3sim::replay2
