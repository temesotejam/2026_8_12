#include "BoatLog2Replay.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

#include "boat_protocol.h"

namespace cores3sim::replay2 {
namespace {

template <typename T>
BoatLog2RecordRaw makeRecord(RecordType type, std::uint32_t sequence,
                            std::uint64_t at_us, const T* payload) {
  static_assert(sizeof(T) <= kPayloadBytes, "BOATLOG2 payload too large");
  BoatLog2RecordRaw record{};
  record.type = static_cast<std::uint8_t>(type);
  record.payload_bytes = payload ? static_cast<std::uint16_t>(sizeof(T)) : 0;
  record.sequence = sequence;
  record.at_us = at_us;
  if (payload) std::memcpy(record.payload, payload, sizeof(T));
  record.record_crc32 = recordCrc(record);
  return record;
}

BoatLog2RecordRaw makeEmptyRecord(RecordType type, std::uint32_t sequence,
                                  std::uint64_t at_us) {
  BoatLog2RecordRaw record{};
  record.type = static_cast<std::uint8_t>(type);
  record.sequence = sequence;
  record.at_us = at_us;
  record.record_crc32 = recordCrc(record);
  return record;
}

template <typename T>
bool payloadAs(const BoatLog2RecordRaw& record, T& out) {
  if (record.payload_bytes != sizeof(T)) return false;
  std::memcpy(&out, record.payload, sizeof(T));
  return true;
}

bool validSafety(std::uint8_t value) { return value <= 5; }
bool validMode(std::uint8_t value) { return value <= 4; }
bool validAck(std::uint8_t value) { return value <= 5; }

CommandResultWire encodeResult(production_control::CommandResult result) {
  CommandResultWire out{};
  out.ack = static_cast<std::uint8_t>(result.ack);
  out.reason = result.reason;
  return out;
}

production_control::CommandResult decodeResult(const CommandResultWire& wire) {
  return {static_cast<production_control::Ack>(wire.ack), wire.reason};
}

bool resultMatches(production_control::CommandResult actual,
                   const CommandResultWire& expected) {
  return static_cast<std::uint8_t>(actual.ack) == expected.ack &&
         actual.reason == expected.reason;
}

void rememberFailure(ReplayReport& report, std::uint32_t sequence,
                     const std::string& field) {
  if (report.first_failure_sequence == 0) {
    report.first_failure_sequence = sequence;
    report.first_failure_field = field;
  }
}

bool sameFloat(float actual, float expected, float epsilon, float& max_error) {
  if (std::isnan(actual) || std::isnan(expected)) {
    return std::isnan(actual) && std::isnan(expected);
  }
  if (std::isinf(actual) || std::isinf(expected)) return actual == expected;
  const float error = std::fabs(actual - expected);
  max_error = std::max(max_error, error);
  return error <= epsilon;
}

bool compareOutput(const production_control::Output& actual,
                   const OutputWire& expected, float epsilon,
                   float& max_error, std::string& failure) {
#define CMP_FLOAT(member, wire_member) \
  do { if (!sameFloat(actual.member, expected.wire_member, epsilon, max_error)) { \
    if (failure.empty()) failure = #member; match = false; } } while (0)
#define CMP_EXACT(expr, expected_expr, name) \
  do { if ((expr) != (expected_expr)) { if (failure.empty()) failure = name; match = false; } } while (0)
  bool match = true;
  CMP_FLOAT(leftFront, left_front);
  CMP_FLOAT(rightFront, right_front);
  CMP_FLOAT(rearYaw, rear_yaw);
  CMP_FLOAT(propulsion, propulsion);
  CMP_FLOAT(leftPrelimit, left_prelimit);
  CMP_FLOAT(rightPrelimit, right_prelimit);
  CMP_FLOAT(rearPrelimit, rear_prelimit);
  CMP_FLOAT(propulsionPrelimit, propulsion_prelimit);
  CMP_FLOAT(uHeight, u_height);
  CMP_FLOAT(uPitch, u_pitch);
  CMP_FLOAT(uRoll, u_roll);
  CMP_FLOAT(uYaw, u_yaw);
  CMP_FLOAT(targetYaw, target_yaw);
  CMP_FLOAT(courseErrorRad, course_error_rad);
  CMP_FLOAT(waypointDistanceM, waypoint_distance_m);
  CMP_FLOAT(throttleLimit, throttle_limit);
  CMP_EXACT(static_cast<std::uint8_t>(actual.safety), expected.safety, "safety");
  CMP_EXACT(static_cast<std::uint8_t>(actual.mode), expected.mode, "mode");
  CMP_EXACT(static_cast<std::uint8_t>(actual.reason), expected.reason, "reason");
  CMP_EXACT(static_cast<std::uint8_t>(actual.safetyRequest), expected.safety_request,
            "safetyRequest");
  CMP_EXACT(actual.flags, expected.flags, "flags");
  CMP_EXACT(actual.activeWaypoint, expected.active_waypoint, "activeWaypoint");
  CMP_EXACT(actual.enabledMask, expected.enabled_mask, "enabledMask");
  CMP_EXACT(actual.saturated ? 1u : 0u, expected.saturated, "saturated");
  CMP_EXACT(actual.physicalGate ? 1u : 0u, expected.physical_gate, "physicalGate");
  CMP_EXACT(actual.waypointReached ? 1u : 0u, expected.waypoint_reached,
            "waypointReached");
#undef CMP_FLOAT
#undef CMP_EXACT
  return match;
}

const char* typeName(RecordType type) {
  switch (type) {
    case RecordType::Reset: return "Reset";
    case RecordType::SetConfig: return "SetConfig";
    case RecordType::SetMode: return "SetMode";
    case RecordType::SetManual: return "SetManual";
    case RecordType::SetHeading: return "SetHeading";
    case RecordType::SetWaypoints: return "SetWaypoints";
    case RecordType::SetWaypointReachRadius: return "SetWaypointReachRadius";
    case RecordType::Step: return "Step";
  }
  return "Unknown";
}

bool knownType(std::uint8_t raw) {
  return raw >= static_cast<std::uint8_t>(RecordType::Reset) &&
         raw <= static_cast<std::uint8_t>(RecordType::Step);
}

std::string jsonEscape(const std::string& value) {
  std::string out;
  for (char ch : value) {
    if (ch == '\\' || ch == '"') out.push_back('\\');
    if (ch == '\n') { out += "\\n"; continue; }
    out.push_back(ch);
  }
  return out;
}

}  // namespace

ConfigWire encodeConfig(const production_control::Config& c) {
  ConfigWire w{};
  w.kp_pitch=c.kpPitch; w.kd_pitch=c.kdPitch; w.kp_roll=c.kpRoll; w.kd_roll=c.kdRoll;
  w.kp_height=c.kpHeight; w.kp_yaw=c.kpYaw; w.kd_yaw=c.kdYaw;
  w.target_pitch=c.targetPitch; w.target_roll=c.targetRoll; w.target_height_m=c.targetHeightM;
  w.attitude_servo_limit=c.attitudeServoLimit;
  w.auto_propulsion=c.autoPropulsion; w.slew_per_step=c.slewPerStep; w.waypoint_reach_m=c.waypointReachM;
  w.los_lookahead_m=c.losLookaheadM; w.min_course_speed_mps=c.minCourseSpeedMps;
  w.pitch_priority_rad=c.pitchPriorityRad; w.attitude_stop_rad=c.attitudeStopRad;
  w.enable_attitude_danger_trip=c.enableAttitudeDangerTrip?1:0;
  w.low_speed_mps=c.lowSpeedMps; w.high_speed_mps=c.highSpeedMps;
  w.high_speed_yaw_gain=c.highSpeedYawGain; w.high_speed_yaw_limit=c.highSpeedYawLimit;
  w.low_voltage_v=c.lowVoltageV; w.critical_voltage_v=c.criticalVoltageV;
  w.over_current_a=c.overCurrentA; w.critical_current_a=c.criticalCurrentA;
  w.stall_current_a=c.stallCurrentA; w.stall_erpm=c.stallErpm; w.stall_command=c.stallCommand;
  w.cavitation_erpm=c.cavitationErpm; w.cavitation_current_a=c.cavitationCurrentA;
  w.cavitation_speed_mps=c.cavitationSpeedMps;
  w.heartbeat_stale_us=c.heartbeatStaleUs; w.imu_stale_us=c.imuStaleUs;
  w.tof_stale_us=c.tofStaleUs; w.gnss_stale_us=c.gnssStaleUs;
  w.manual_stale_us=c.manualStaleUs; w.power_stale_us=c.powerStaleUs;
  w.vesc_stale_us=c.vescStaleUs; w.stall_trip_us=c.stallTripUs;
  const auto& p=c.physical;
  w.physical.left_channel=p.leftChannel; w.physical.right_channel=p.rightChannel;
  w.physical.rear_channel=p.rearChannel; w.physical.propulsion_channel=p.propulsionChannel;
  w.physical.left_min_us=p.leftMinUs; w.physical.left_center_us=p.leftCenterUs; w.physical.left_max_us=p.leftMaxUs;
  w.physical.right_min_us=p.rightMinUs; w.physical.right_center_us=p.rightCenterUs; w.physical.right_max_us=p.rightMaxUs;
  w.physical.rear_min_us=p.rearMinUs; w.physical.rear_center_us=p.rearCenterUs; w.physical.rear_max_us=p.rearMaxUs;
  w.physical.prop_min_us=p.propMinUs; w.physical.prop_stop_us=p.propStopUs; w.physical.prop_max_us=p.propMaxUs;
  w.physical.calibration_complete=p.calibrationComplete?1:0;
  return w;
}

production_control::Config decodeConfig(const ConfigWire& w) {
  production_control::Config c{};
  c.kpPitch=w.kp_pitch; c.kdPitch=w.kd_pitch; c.kpRoll=w.kp_roll; c.kdRoll=w.kd_roll;
  c.kpHeight=w.kp_height; c.kpYaw=w.kp_yaw; c.kdYaw=w.kd_yaw;
  c.targetPitch=w.target_pitch; c.targetRoll=w.target_roll; c.targetHeightM=w.target_height_m;
  c.attitudeServoLimit=w.attitude_servo_limit;
  c.autoPropulsion=w.auto_propulsion; c.slewPerStep=w.slew_per_step; c.waypointReachM=w.waypoint_reach_m;
  c.losLookaheadM=w.los_lookahead_m; c.minCourseSpeedMps=w.min_course_speed_mps;
  c.pitchPriorityRad=w.pitch_priority_rad; c.attitudeStopRad=w.attitude_stop_rad;
  c.enableAttitudeDangerTrip=w.enable_attitude_danger_trip!=0;
  c.lowSpeedMps=w.low_speed_mps; c.highSpeedMps=w.high_speed_mps;
  c.highSpeedYawGain=w.high_speed_yaw_gain; c.highSpeedYawLimit=w.high_speed_yaw_limit;
  c.lowVoltageV=w.low_voltage_v; c.criticalVoltageV=w.critical_voltage_v;
  c.overCurrentA=w.over_current_a; c.criticalCurrentA=w.critical_current_a;
  c.stallCurrentA=w.stall_current_a; c.stallErpm=w.stall_erpm; c.stallCommand=w.stall_command;
  c.cavitationErpm=w.cavitation_erpm; c.cavitationCurrentA=w.cavitation_current_a;
  c.cavitationSpeedMps=w.cavitation_speed_mps;
  c.heartbeatStaleUs=w.heartbeat_stale_us; c.imuStaleUs=w.imu_stale_us;
  c.tofStaleUs=w.tof_stale_us; c.gnssStaleUs=w.gnss_stale_us;
  c.manualStaleUs=w.manual_stale_us; c.powerStaleUs=w.power_stale_us;
  c.vescStaleUs=w.vesc_stale_us; c.stallTripUs=w.stall_trip_us;
  auto& p=c.physical;
  p.leftChannel=w.physical.left_channel; p.rightChannel=w.physical.right_channel;
  p.rearChannel=w.physical.rear_channel; p.propulsionChannel=w.physical.propulsion_channel;
  p.leftMinUs=w.physical.left_min_us; p.leftCenterUs=w.physical.left_center_us; p.leftMaxUs=w.physical.left_max_us;
  p.rightMinUs=w.physical.right_min_us; p.rightCenterUs=w.physical.right_center_us; p.rightMaxUs=w.physical.right_max_us;
  p.rearMinUs=w.physical.rear_min_us; p.rearCenterUs=w.physical.rear_center_us; p.rearMaxUs=w.physical.rear_max_us;
  p.propMinUs=w.physical.prop_min_us; p.propStopUs=w.physical.prop_stop_us; p.propMaxUs=w.physical.prop_max_us;
  p.calibrationComplete=w.physical.calibration_complete!=0;
  return c;
}

SensorInputWire encodeSensorInput(const production_control::SensorInput& s) {
  SensorInputWire w{};
  w.heartbeat=s.heartbeat?1:0; w.imu_valid=s.imuValid?1:0; w.tof_valid=s.tofValid?1:0;
  w.gnss_valid=s.gnssValid?1:0; w.power_valid=s.powerValid?1:0; w.vesc_valid=s.vescValid?1:0;
  w.vesc_fault=s.vescFault?1:0; w.safety=static_cast<std::uint8_t>(s.safety);
  w.now_us=s.nowUs; w.heartbeat_us=s.heartbeatUs; w.imu_us=s.imuUs; w.tof_us=s.tofUs;
  w.gnss_us=s.gnssUs; w.power_us=s.powerUs; w.vesc_us=s.vescUs;
  w.roll_rad=s.rollRad; w.pitch_rad=s.pitchRad; w.yaw_rad=s.yawRad;
  w.roll_rate_rad_s=s.rollRateRadS; w.pitch_rate_rad_s=s.pitchRateRadS; w.yaw_rate_rad_s=s.yawRateRadS;
  w.tof_m=s.tofM; w.north_m=s.northM; w.east_m=s.eastM; w.ground_speed_mps=s.groundSpeedMps;
  w.course_rad=s.courseRad; w.bus_voltage_v=s.busVoltageV; w.current_a=s.currentA;
  w.power_w=s.powerW; w.vesc_erpm=s.vescErpm;
  return w;
}

production_control::SensorInput decodeSensorInput(const SensorInputWire& w) {
  production_control::SensorInput s{};
  s.heartbeat=w.heartbeat!=0; s.imuValid=w.imu_valid!=0; s.tofValid=w.tof_valid!=0;
  s.gnssValid=w.gnss_valid!=0; s.powerValid=w.power_valid!=0; s.vescValid=w.vesc_valid!=0;
  s.vescFault=w.vesc_fault!=0; s.safety=static_cast<production_control::AuthoritativeSafety>(w.safety);
  s.nowUs=w.now_us; s.heartbeatUs=w.heartbeat_us; s.imuUs=w.imu_us; s.tofUs=w.tof_us;
  s.gnssUs=w.gnss_us; s.powerUs=w.power_us; s.vescUs=w.vesc_us;
  s.rollRad=w.roll_rad; s.pitchRad=w.pitch_rad; s.yawRad=w.yaw_rad;
  s.rollRateRadS=w.roll_rate_rad_s; s.pitchRateRadS=w.pitch_rate_rad_s; s.yawRateRadS=w.yaw_rate_rad_s;
  s.tofM=w.tof_m; s.northM=w.north_m; s.eastM=w.east_m; s.groundSpeedMps=w.ground_speed_mps;
  s.courseRad=w.course_rad; s.busVoltageV=w.bus_voltage_v; s.currentA=w.current_a;
  s.powerW=w.power_w; s.vescErpm=w.vesc_erpm;
  return s;
}

OutputWire encodeOutput(const production_control::Output& o) {
  OutputWire w{};
  w.left_front=o.leftFront; w.right_front=o.rightFront; w.rear_yaw=o.rearYaw; w.propulsion=o.propulsion;
  w.left_prelimit=o.leftPrelimit; w.right_prelimit=o.rightPrelimit; w.rear_prelimit=o.rearPrelimit; w.propulsion_prelimit=o.propulsionPrelimit;
  w.u_height=o.uHeight; w.u_pitch=o.uPitch; w.u_roll=o.uRoll; w.u_yaw=o.uYaw; w.target_yaw=o.targetYaw;
  w.course_error_rad=o.courseErrorRad; w.waypoint_distance_m=o.waypointDistanceM; w.throttle_limit=o.throttleLimit;
  w.safety=static_cast<std::uint8_t>(o.safety); w.mode=static_cast<std::uint8_t>(o.mode);
  w.reason=static_cast<std::uint8_t>(o.reason); w.safety_request=static_cast<std::uint8_t>(o.safetyRequest);
  w.flags=o.flags; w.active_waypoint=o.activeWaypoint; w.enabled_mask=o.enabledMask;
  w.saturated=o.saturated?1:0; w.physical_gate=o.physicalGate?1:0; w.waypoint_reached=o.waypointReached?1:0;
  return w;
}

production_control::Output decodeOutput(const OutputWire& w) {
  production_control::Output o{};
  o.leftFront=w.left_front; o.rightFront=w.right_front; o.rearYaw=w.rear_yaw; o.propulsion=w.propulsion;
  o.leftPrelimit=w.left_prelimit; o.rightPrelimit=w.right_prelimit; o.rearPrelimit=w.rear_prelimit; o.propulsionPrelimit=w.propulsion_prelimit;
  o.uHeight=w.u_height; o.uPitch=w.u_pitch; o.uRoll=w.u_roll; o.uYaw=w.u_yaw; o.targetYaw=w.target_yaw;
  o.courseErrorRad=w.course_error_rad; o.waypointDistanceM=w.waypoint_distance_m; o.throttleLimit=w.throttle_limit;
  o.safety=static_cast<production_control::AuthoritativeSafety>(w.safety);
  o.mode=static_cast<production_control::ControlMode>(w.mode);
  o.reason=static_cast<production_control::StopReason>(w.reason);
  o.safetyRequest=static_cast<production_control::SafetyRequest>(w.safety_request);
  o.flags=w.flags; o.activeWaypoint=w.active_waypoint; o.enabledMask=w.enabled_mask;
  o.saturated=w.saturated!=0; o.physicalGate=w.physical_gate!=0; o.waypointReached=w.waypoint_reached!=0;
  return o;
}

std::uint32_t recordCrc(const BoatLog2RecordRaw& record) {
  return boat::crc32(reinterpret_cast<const std::uint8_t*>(&record),
                     offsetof(BoatLog2RecordRaw, record_crc32));
}

BoatLog2RecordRaw makeResetRecord(std::uint32_t sequence, std::uint64_t at_us) {
  return makeEmptyRecord(RecordType::Reset, sequence, at_us);
}

BoatLog2RecordRaw makeConfigRecord(std::uint32_t sequence, std::uint64_t at_us,
                                   const production_control::Config& config) {
  const ConfigWire wire=encodeConfig(config);
  return makeRecord(RecordType::SetConfig, sequence, at_us, &wire);
}

BoatLog2RecordRaw makeModeRecord(std::uint32_t sequence, std::uint64_t at_us,
                                 production_control::ControlMode mode,
                                 std::uint32_t request_id,
                                 production_control::AuthoritativeSafety safety,
                                 production_control::CommandResult expected) {
  SetModeWire wire{}; wire.mode=static_cast<std::uint8_t>(mode); wire.safety=static_cast<std::uint8_t>(safety);
  wire.request_id=request_id; wire.expected=encodeResult(expected);
  return makeRecord(RecordType::SetMode, sequence, at_us, &wire);
}

BoatLog2RecordRaw makeManualRecord(std::uint32_t sequence, std::uint64_t at_us,
                                   const production_control::ManualCommand& command,
                                   std::uint64_t received_us,
                                   production_control::CommandResult expected) {
  SetManualWire wire{}; wire.left_front=command.leftFront; wire.right_front=command.rightFront;
  wire.rear_yaw=command.rearYaw; wire.propulsion=command.propulsion; wire.enabled_mask=command.enabledMask;
  wire.expected=encodeResult(expected); wire.received_us=received_us;
  return makeRecord(RecordType::SetManual, sequence, at_us, &wire);
}

BoatLog2RecordRaw makeHeadingRecord(std::uint32_t sequence, std::uint64_t at_us,
                                    float yaw_rad, std::uint32_t request_id,
                                    production_control::CommandResult expected) {
  SetHeadingWire wire{}; wire.yaw_rad=yaw_rad; wire.request_id=request_id; wire.expected=encodeResult(expected);
  return makeRecord(RecordType::SetHeading, sequence, at_us, &wire);
}

BoatLog2RecordRaw makeWaypointsRecord(std::uint32_t sequence, std::uint64_t at_us,
                                      const production_control::Waypoint* points,
                                      std::uint8_t count, std::uint32_t request_id,
                                      production_control::AuthoritativeSafety safety,
                                      production_control::CommandResult expected) {
  SetWaypointsWire wire{}; wire.request_id=request_id; wire.safety=static_cast<std::uint8_t>(safety);
  wire.count=count; wire.expected=encodeResult(expected);
  if (points) for (std::uint8_t i=0;i<count && i<16;++i) { wire.points[i].north_m=points[i].northM; wire.points[i].east_m=points[i].eastM; }
  return makeRecord(RecordType::SetWaypoints, sequence, at_us, &wire);
}

BoatLog2RecordRaw makeReachRecord(std::uint32_t sequence, std::uint64_t at_us,
                                  float radius_m,
                                  production_control::AuthoritativeSafety safety,
                                  production_control::CommandResult expected) {
  SetReachWire wire{}; wire.radius_m=radius_m; wire.safety=static_cast<std::uint8_t>(safety);
  wire.expected=encodeResult(expected);
  return makeRecord(RecordType::SetWaypointReachRadius, sequence, at_us, &wire);
}

BoatLog2RecordRaw makeStepRecord(std::uint32_t sequence, std::uint64_t at_us,
                                 const production_control::SensorInput& input,
                                 const production_control::Output& expected) {
  StepWire wire{}; wire.input=encodeSensorInput(input); wire.expected=encodeOutput(expected);
  return makeRecord(RecordType::Step, sequence, at_us, &wire);
}

bool readBoatLog2(const std::filesystem::path& path, BoatLog2File& out,
                  std::string& error) {
  out={}; error.clear();
  std::ifstream file(path, std::ios::binary);
  if (!file) { error="cannot open BOATLOG2 file"; return false; }
  file.read(reinterpret_cast<char*>(&out.header), sizeof(out.header));
  if (file.gcount()!=static_cast<std::streamsize>(sizeof(out.header))) { error="BOATLOG2 header truncated"; return false; }
  if (std::memcmp(out.header.magic,"BOATLOG2",8)!=0) { error="BOATLOG2 magic mismatch"; return false; }
  if (out.header.version!=kVersion) { error="BOATLOG2 version mismatch"; return false; }
  if (out.header.header_bytes!=kHeaderBytes) { error="BOATLOG2 header size mismatch"; return false; }
  if (out.header.record_bytes!=kRecordBytes) { error="BOATLOG2 record size mismatch"; return false; }
  if (out.header.endian_tag!=kLittleEndianTag) { error="BOATLOG2 endian marker mismatch"; return false; }
  if (out.header.controller_abi_id!=kControllerAbiId) { error="BOATLOG2 controller ABI mismatch"; return false; }
  while (true) {
    BoatLog2RecordRaw record{};
    file.read(reinterpret_cast<char*>(&record), sizeof(record));
    const auto got=file.gcount();
    if (got==0) break;
    if (got!=static_cast<std::streamsize>(sizeof(record))) {
      out.trailing_bytes=static_cast<std::uint32_t>(got);
      break;
    }
    out.records.push_back(record);
  }
  return true;
}

bool writeBoatLog2(const std::filesystem::path& path,
                   const BoatLog2HeaderRaw& header,
                   const std::vector<BoatLog2RecordRaw>& records,
                   std::string& error) {
  error.clear();
  std::ofstream file(path,std::ios::binary|std::ios::trunc);
  if(!file){error="cannot create BOATLOG2 file";return false;}
  file.write(reinterpret_cast<const char*>(&header),sizeof(header));
  for(const auto& record:records) file.write(reinterpret_cast<const char*>(&record),sizeof(record));
  if(!file){error="failed writing BOATLOG2 file";return false;}
  return true;
}

ReplayReport replay(const BoatLog2File& log, float epsilon) {
  ReplayReport report{}; report.total_records=log.records.size(); report.trailing_bytes=log.trailing_bytes;
  production_control::Controller controller{};
  std::uint32_t expected_sequence=1;
  for(const auto& record:log.records){
    ReplayRow row{}; row.sequence=record.sequence;
    if(record.sequence!=expected_sequence){
      ++report.sequence_failures; row.match=false; row.failure="sequence"; rememberFailure(report,record.sequence,"sequence"); report.rows.push_back(row); break;
    }
    ++expected_sequence;
    if(record.record_crc32!=recordCrc(record)){
      ++report.crc_failures; row.match=false; row.failure="record_crc32"; rememberFailure(report,record.sequence,"record_crc32"); report.rows.push_back(row); break;
    }
    if(!knownType(record.type)){
      ++report.unknown_records; row.match=false; row.failure="record_type"; rememberFailure(report,record.sequence,"record_type"); report.rows.push_back(row); break;
    }
    const auto type=static_cast<RecordType>(record.type); row.type=type;
    bool malformed=false;
    switch(type){
      case RecordType::Reset:
        if(record.payload_bytes!=0) malformed=true; else controller.reset();
        break;
      case RecordType::SetConfig:{
        ConfigWire wire{}; if(!payloadAs(record,wire)){malformed=true;break;} controller.setConfig(decodeConfig(wire)); break;}
      case RecordType::SetMode:{
        SetModeWire wire{}; if(!payloadAs(record,wire)||!validMode(wire.mode)||!validSafety(wire.safety)||!validAck(wire.expected.ack)){malformed=true;break;}
        const auto actual=controller.setMode(static_cast<production_control::ControlMode>(wire.mode),wire.request_id,static_cast<production_control::AuthoritativeSafety>(wire.safety));
        if(!resultMatches(actual,wire.expected)){++report.command_mismatches;row.match=false;row.failure="SetMode.CommandResult";rememberFailure(report,record.sequence,row.failure);} break;}
      case RecordType::SetManual:{
        SetManualWire wire{}; if(!payloadAs(record,wire)||!validAck(wire.expected.ack)){malformed=true;break;}
        production_control::ManualCommand command{wire.left_front,wire.right_front,wire.rear_yaw,wire.propulsion,wire.enabled_mask};
        const auto actual=controller.setManual(command,wire.received_us);
        if(!resultMatches(actual,wire.expected)){++report.command_mismatches;row.match=false;row.failure="SetManual.CommandResult";rememberFailure(report,record.sequence,row.failure);} break;}
      case RecordType::SetHeading:{
        SetHeadingWire wire{}; if(!payloadAs(record,wire)||!validAck(wire.expected.ack)){malformed=true;break;}
        const auto actual=controller.setHeading(wire.yaw_rad,wire.request_id);
        if(!resultMatches(actual,wire.expected)){++report.command_mismatches;row.match=false;row.failure="SetHeading.CommandResult";rememberFailure(report,record.sequence,row.failure);} break;}
      case RecordType::SetWaypoints:{
        SetWaypointsWire wire{}; if(!payloadAs(record,wire)||wire.count>16||!validSafety(wire.safety)||!validAck(wire.expected.ack)){malformed=true;break;}
        production_control::Waypoint points[16]{}; for(std::uint8_t i=0;i<wire.count;++i){points[i].northM=wire.points[i].north_m;points[i].eastM=wire.points[i].east_m;}
        const auto actual=controller.setWaypoints(points,wire.count,wire.request_id,static_cast<production_control::AuthoritativeSafety>(wire.safety));
        if(!resultMatches(actual,wire.expected)){++report.command_mismatches;row.match=false;row.failure="SetWaypoints.CommandResult";rememberFailure(report,record.sequence,row.failure);} break;}
      case RecordType::SetWaypointReachRadius:{
        SetReachWire wire{}; if(!payloadAs(record,wire)||!validSafety(wire.safety)||!validAck(wire.expected.ack)){malformed=true;break;}
        const auto actual=controller.setWaypointReachRadius(wire.radius_m,static_cast<production_control::AuthoritativeSafety>(wire.safety));
        if(!resultMatches(actual,wire.expected)){++report.command_mismatches;row.match=false;row.failure="SetWaypointReachRadius.CommandResult";rememberFailure(report,record.sequence,row.failure);} break;}
      case RecordType::Step:{
        StepWire wire{}; if(!payloadAs(record,wire)||!validSafety(wire.input.safety)||!validSafety(wire.expected.safety)||!validMode(wire.expected.mode)){malformed=true;break;}
        const auto actual=controller.step(decodeSensorInput(wire.input));
        std::string failure; float max_error=0.0f;
        if(!compareOutput(actual,wire.expected,epsilon,max_error,failure)){++report.step_mismatches;row.match=false;row.failure="Step."+failure;rememberFailure(report,record.sequence,row.failure);}
        row.max_float_error=max_error; report.max_float_error=std::max(report.max_float_error,max_error); break;}
    }
    if(malformed){++report.malformed_records;row.match=false;row.failure="payload";rememberFailure(report,record.sequence,"payload");report.rows.push_back(row);break;}
    ++report.processed_records; report.rows.push_back(row);
  }
  report.pass=!log.records.empty()&&report.processed_records==log.records.size()&&
      report.crc_failures==0&&report.sequence_failures==0&&report.malformed_records==0&&
      report.unknown_records==0&&report.command_mismatches==0&&report.step_mismatches==0;
  return report;
}

bool writeReplayCsv(const std::filesystem::path& path,const ReplayReport& report,std::string& error){
  error.clear();std::ofstream out(path);if(!out){error="cannot write replay CSV";return false;}
  out<<"sequence,type,match,max_float_error,failure\n";
  for(const auto& row:report.rows) out<<row.sequence<<','<<typeName(row.type)<<','<<(row.match?1:0)<<','<<std::setprecision(9)<<row.max_float_error<<','<<row.failure<<'\n';
  return static_cast<bool>(out);
}

bool writeReplayJson(const std::filesystem::path& path,const ReplayReport& r,float epsilon,std::string& error){
  error.clear();std::ofstream out(path);if(!out){error="cannot write replay JSON";return false;}
  out<<"{\n  \"pass\":"<<(r.pass?"true":"false")<<",\n"
     <<"  \"float_epsilon\":"<<std::setprecision(9)<<epsilon<<",\n"
     <<"  \"total_records\":"<<r.total_records<<",\n"
     <<"  \"processed_records\":"<<r.processed_records<<",\n"
     <<"  \"trailing_bytes\":"<<r.trailing_bytes<<",\n"
     <<"  \"crc_failures\":"<<r.crc_failures<<",\n"
     <<"  \"sequence_failures\":"<<r.sequence_failures<<",\n"
     <<"  \"malformed_records\":"<<r.malformed_records<<",\n"
     <<"  \"unknown_records\":"<<r.unknown_records<<",\n"
     <<"  \"command_mismatches\":"<<r.command_mismatches<<",\n"
     <<"  \"step_mismatches\":"<<r.step_mismatches<<",\n"
     <<"  \"max_float_error\":"<<r.max_float_error<<",\n"
     <<"  \"first_failure_sequence\":"<<r.first_failure_sequence<<",\n"
     <<"  \"first_failure_field\":\""<<jsonEscape(r.first_failure_field)<<"\"\n}\n";
  return static_cast<bool>(out);
}

bool writeReplaySvg(const std::filesystem::path& path,const ReplayReport& r,float epsilon,std::string& error){
  error.clear();std::ofstream out(path);if(!out){error="cannot write replay SVG";return false;}
  out<<"<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"760\" height=\"360\">"
     <<"<rect width=\"760\" height=\"360\" fill=\"#101418\"/>"
     <<"<g fill=\"white\" font-family=\"monospace\">"
     <<"<text x=\"24\" y=\"36\" font-size=\"22\">Phase 11 BOATLOG2 deterministic replay</text>"
     <<"<text x=\"24\" y=\"78\" font-size=\"26\">"<<(r.pass?"PASS":"FAIL")<<"</text>"
     <<"<text x=\"24\" y=\"112\" font-size=\"14\">records "<<r.processed_records<<" / "<<r.total_records<<"  trailing="<<r.trailing_bytes<<"B</text>"
     <<"<text x=\"24\" y=\"140\" font-size=\"14\">CRC="<<r.crc_failures<<" sequence="<<r.sequence_failures<<" malformed="<<r.malformed_records<<" unknown="<<r.unknown_records<<"</text>"
     <<"<text x=\"24\" y=\"168\" font-size=\"14\">command mismatch="<<r.command_mismatches<<" step mismatch="<<r.step_mismatches<<"</text>"
     <<"<text x=\"24\" y=\"196\" font-size=\"14\">max float error="<<std::setprecision(9)<<r.max_float_error<<" epsilon="<<epsilon<<"</text>"
     <<"<text x=\"24\" y=\"224\" font-size=\"14\">first failure seq="<<r.first_failure_sequence<<" field="<<r.first_failure_field<<"</text>"
     <<"<text x=\"24\" y=\"278\" font-size=\"13\">controller ABI 0xBF8067C3 | record 320B | CRC32 per record</text>"
     <<"<text x=\"24\" y=\"306\" font-size=\"13\">reset/config/commands/waypoints/SensorInput -> production Controller::step()</text>"
     <<"</g></svg>";
  return static_cast<bool>(out);
}

}  // namespace cores3sim::replay2
