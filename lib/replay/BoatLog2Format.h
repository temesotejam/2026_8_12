#pragma once

#include <cstddef>
#include <cstdint>

namespace cores3sim::replay2 {

constexpr std::uint16_t kVersion = 2;
constexpr std::uint16_t kHeaderBytes = 32;
constexpr std::uint16_t kRecordBytes = 320;
constexpr std::uint16_t kLittleEndianTag = 0x4c45;  // ASCII "LE".
constexpr std::uint32_t kControllerAbiId = 0xbf8067c3u;
constexpr std::size_t kPayloadBytes = 300;
constexpr float kDefaultFloatEpsilon = 1.0e-5f;

enum class RecordType : std::uint8_t {
  Reset = 1,
  SetConfig = 2,
  SetMode = 3,
  SetManual = 4,
  SetHeading = 5,
  SetWaypoints = 6,
  SetWaypointReachRadius = 7,
  Step = 8,
};

#pragma pack(push, 1)
struct BoatLog2HeaderRaw {
  char magic[8]{'B','O','A','T','L','O','G','2'};
  std::uint16_t version{kVersion};
  std::uint16_t header_bytes{kHeaderBytes};
  std::uint16_t record_bytes{kRecordBytes};
  std::uint16_t endian_tag{kLittleEndianTag};
  std::uint32_t controller_abi_id{kControllerAbiId};
  std::uint32_t boot_id{0};
  std::uint64_t start_us{0};
};

struct BoatLog2RecordRaw {
  std::uint8_t type{0};
  std::uint8_t flags{0};
  std::uint16_t payload_bytes{0};
  std::uint32_t sequence{0};
  std::uint64_t at_us{0};
  std::uint8_t payload[kPayloadBytes]{};
  std::uint32_t record_crc32{0};
};

struct PhysicalConfigWire {
  std::uint8_t left_channel{0};
  std::uint8_t right_channel{1};
  std::uint8_t rear_channel{2};
  std::uint8_t propulsion_channel{3};
  float left_min_us{1200};
  float left_center_us{1500};
  float left_max_us{1800};
  float right_min_us{1200};
  float right_center_us{1500};
  float right_max_us{1800};
  float rear_min_us{1200};
  float rear_center_us{1500};
  float rear_max_us{1800};
  float prop_min_us{1000};
  float prop_stop_us{1100};
  float prop_max_us{2000};
  std::uint8_t calibration_complete{1};
};

struct ConfigWire {
  float kp_pitch{0.8f}, kd_pitch{0.10f}, kp_roll{1.25f}, kd_roll{0.22f};
  float kp_height{0.75f}, kp_yaw{0.90f}, kd_yaw{0.12f};
  float target_pitch{0}, target_roll{0}, target_height_m{0.45f};
  float attitude_servo_limit{0.50f};
  float auto_propulsion{0.55f}, slew_per_step{0.04f}, waypoint_reach_m{1.5f};
  float los_lookahead_m{4.0f}, min_course_speed_mps{0.5f};
  float pitch_priority_rad{0.35f}, attitude_stop_rad{0.61f};
  std::uint8_t enable_attitude_danger_trip{0};
  float low_speed_mps{0.5f}, high_speed_mps{3.0f};
  float high_speed_yaw_gain{0.45f}, high_speed_yaw_limit{0.35f};
  float low_voltage_v{9.5f}, critical_voltage_v{8.5f};
  float over_current_a{22.0f}, critical_current_a{28.0f};
  float stall_current_a{8.0f}, stall_erpm{100.0f}, stall_command{0.25f};
  float cavitation_erpm{4500.0f}, cavitation_current_a{2.0f}, cavitation_speed_mps{0.35f};
  std::uint32_t heartbeat_stale_us{500000};
  std::uint32_t imu_stale_us{100000};
  std::uint32_t tof_stale_us{250000};
  std::uint32_t gnss_stale_us{500000};
  std::uint32_t manual_stale_us{500000};
  std::uint32_t power_stale_us{200000};
  std::uint32_t vesc_stale_us{300000};
  std::uint32_t stall_trip_us{1000000};
  PhysicalConfigWire physical{};
};

struct CommandResultWire {
  std::uint8_t ack{0};
  std::uint16_t reason{0};
};

struct SetModeWire {
  std::uint8_t mode{0};
  std::uint8_t safety{1};
  std::uint16_t reserved{0};
  std::uint32_t request_id{0};
  CommandResultWire expected{};
};

struct SetManualWire {
  float left_front{0}, right_front{0}, rear_yaw{0}, propulsion{0};
  std::uint8_t enabled_mask{0};
  std::uint8_t reserved{0};
  CommandResultWire expected{};
  std::uint64_t received_us{0};
};

struct SetHeadingWire {
  float yaw_rad{0};
  std::uint32_t request_id{0};
  CommandResultWire expected{};
};

struct WaypointWire { float north_m{0}, east_m{0}; };

struct SetWaypointsWire {
  std::uint32_t request_id{0};
  std::uint8_t safety{1};
  std::uint8_t count{0};
  CommandResultWire expected{};
  WaypointWire points[16]{};
};

struct SetReachWire {
  float radius_m{1.5f};
  std::uint8_t safety{1};
  CommandResultWire expected{};
};

struct SensorInputWire {
  std::uint8_t heartbeat{0}, imu_valid{0}, tof_valid{0}, gnss_valid{0};
  std::uint8_t power_valid{0}, vesc_valid{0}, vesc_fault{0}, safety{1};
  std::uint64_t now_us{0}, heartbeat_us{0}, imu_us{0}, tof_us{0}, gnss_us{0};
  std::uint64_t power_us{0}, vesc_us{0};
  float roll_rad{0}, pitch_rad{0}, yaw_rad{0};
  float roll_rate_rad_s{0}, pitch_rate_rad_s{0}, yaw_rate_rad_s{0};
  float tof_m{0}, north_m{0}, east_m{0}, ground_speed_mps{0}, course_rad{0};
  float bus_voltage_v{0}, current_a{0}, power_w{0}, vesc_erpm{0};
};

struct OutputWire {
  float left_front{0}, right_front{0}, rear_yaw{0}, propulsion{0};
  float left_prelimit{0}, right_prelimit{0}, rear_prelimit{0}, propulsion_prelimit{0};
  float u_height{0}, u_pitch{0}, u_roll{0}, u_yaw{0}, target_yaw{0};
  float course_error_rad{0}, waypoint_distance_m{0}, throttle_limit{1.0f};
  std::uint8_t safety{1}, mode{0}, reason{0}, safety_request{0};
  std::uint16_t flags{0};
  std::uint8_t active_waypoint{0}, enabled_mask{0};
  std::uint8_t saturated{0}, physical_gate{0}, waypoint_reached{0};
};

struct StepWire {
  SensorInputWire input{};
  OutputWire expected{};
};
#pragma pack(pop)

static_assert(sizeof(BoatLog2HeaderRaw) == kHeaderBytes, "BOATLOG2 header ABI");
static_assert(sizeof(BoatLog2RecordRaw) == kRecordBytes, "BOATLOG2 record ABI");
static_assert(sizeof(ConfigWire) <= kPayloadBytes, "BOATLOG2 Config payload too large");
static_assert(sizeof(SetWaypointsWire) <= kPayloadBytes, "BOATLOG2 waypoint payload too large");
static_assert(sizeof(StepWire) <= kPayloadBytes, "BOATLOG2 step payload too large");

}  // namespace cores3sim::replay2
