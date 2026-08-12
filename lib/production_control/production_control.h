#pragma once

#include <stdint.h>

namespace production_control {

enum class AuthoritativeSafety : uint8_t { Boot=0, Disarmed=1, ArmedIdle=2, Running=3, EStop=4, Fault=5 };
// Values 0..3 are wire-compatible with the commissioned firmware. WaypointOnly
// is appended so the existing AutoWaypoint meaning is not changed.
enum class ControlMode : uint8_t {
  Manual=0, AttitudeAssist=1, HeadingHold=2, AutoWaypoint=3, WaypointOnly=4
};
constexpr bool modeUsesWaypoint(ControlMode mode){return mode==ControlMode::AutoWaypoint||mode==ControlMode::WaypointOnly;}
constexpr bool modeUsesAttitude(ControlMode mode){return mode==ControlMode::AttitudeAssist||mode==ControlMode::HeadingHold||mode==ControlMode::AutoWaypoint;}
constexpr bool modeNeedsManual(ControlMode mode){return !modeUsesWaypoint(mode);}
enum class Ack : uint8_t { Accepted=0, Rejected=1, Duplicate=2, Conflict=3, Stale=4, Malformed=5 };
enum class SafetyRequest : uint8_t { None=0, Disarm=1, Fault=2 };
enum class StopReason : uint8_t {
  None=0, Stop=1, EStop=2, Heartbeat=3, ManualTimeout=4,
  ImuInvalid=5, ImuStale=6, TofInvalid=7, TofStale=8,
  GnssInvalid=9, GnssStale=10, VescFault=11, NonFinite=12,
  FinalWaypoint=13, PowerInvalid=14, PowerStale=15,
  LowVoltage=16, OverCurrent=17, MotorStall=18,
  AttitudeDanger=19, VescStale=20, Cavitation=21
};

struct CommandResult {
  Ack ack; uint16_t reason;
  constexpr CommandResult(Ack value=Ack::Accepted,uint16_t why=0):ack(value),reason(why){}
};
struct ManualCommand {
  float leftFront,rightFront,rearYaw,propulsion;
  uint8_t enabledMask;
  constexpr ManualCommand(float left=0,float right=0,float rear=0,float thrust=0,uint8_t mask=0):leftFront(left),rightFront(right),rearYaw(rear),propulsion(thrust),enabledMask(mask){}
};
struct Waypoint { float northM=0,eastM=0; };

struct SensorInput {
  bool heartbeat=false,imuValid=false,tofValid=false,gnssValid=false;
  bool powerValid=false,vescValid=false,vescFault=false;
  AuthoritativeSafety safety=AuthoritativeSafety::Disarmed;
  uint64_t nowUs=0,heartbeatUs=0,imuUs=0,tofUs=0,gnssUs=0;
  uint64_t powerUs=0,vescUs=0;
  float rollRad=0,pitchRad=0,yawRad=0;
  float rollRateRadS=0,pitchRateRadS=0,yawRateRadS=0;
  float tofM=0,northM=0,eastM=0,groundSpeedMps=0,courseRad=0;
  float busVoltageV=0,currentA=0,powerW=0,vescErpm=0;
};

struct PhysicalConfig {
  uint8_t leftChannel=0,rightChannel=1,rearChannel=2,propulsionChannel=3;
  float leftMinUs=1200,leftCenterUs=1500,leftMaxUs=1800;
  float rightMinUs=1200,rightCenterUs=1500,rightMaxUs=1800;
  float rearMinUs=1200,rearCenterUs=1500,rearMaxUs=1800;
  float propMinUs=1000,propStopUs=1100,propMaxUs=2000;
  bool calibrationComplete=true;
};

struct Config {
  float kpPitch=.8f,kdPitch=.10f,kpRoll=1.25f,kdRoll=.22f;
  float kpHeight=.75f,kpYaw=.90f,kdYaw=.12f;
  float targetPitch=0,targetRoll=0,targetHeightM=.45f;
  float attitudeServoLimit=.50f;
  float autoPropulsion=.55f,slewPerStep=.04f,waypointReachM=1.5f;
  float losLookaheadM=4.0f,minCourseSpeedMps=.5f;
  float pitchPriorityRad=.35f,attitudeStopRad=.61f;
  bool enableAttitudeDangerTrip=false;
  float lowSpeedMps=.5f,highSpeedMps=3.0f;
  float highSpeedYawGain=.45f,highSpeedYawLimit=.35f;
  float lowVoltageV=9.5f,criticalVoltageV=8.5f;
  float overCurrentA=22.0f,criticalCurrentA=28.0f;
  float stallCurrentA=8.0f,stallErpm=100.0f,stallCommand=.25f;
  float cavitationErpm=4500.0f,cavitationCurrentA=2.0f,cavitationSpeedMps=.35f;
  uint32_t heartbeatStaleUs=500000,imuStaleUs=100000,tofStaleUs=250000;
  uint32_t gnssStaleUs=500000,manualStaleUs=500000,powerStaleUs=200000;
  uint32_t vescStaleUs=300000,stallTripUs=1000000;
  PhysicalConfig physical{};
};

enum OutputFlag : uint16_t {
  HeightDegraded=1u<<0, PitchPriority=1u<<1, VoltageLimited=1u<<2,
  CurrentLimited=1u<<3, CavitationLimited=1u<<4, YawScheduled=1u<<5,
  PropulsionUnavailable=1u<<6
};

enum ManualOutputMask : uint8_t {
  ManualLeft=1u<<0, ManualRight=1u<<1, ManualRear=1u<<2,
  ManualPropulsion=1u<<3, ManualAll=ManualLeft|ManualRight|ManualRear|ManualPropulsion
};

struct Output {
  float leftFront=0,rightFront=0,rearYaw=0,propulsion=0;
  float leftPrelimit=0,rightPrelimit=0,rearPrelimit=0,propulsionPrelimit=0;
  float uHeight=0,uPitch=0,uRoll=0,uYaw=0,targetYaw=0;
  float courseErrorRad=0,waypointDistanceM=0,throttleLimit=1.0f;
  AuthoritativeSafety safety=AuthoritativeSafety::Disarmed;
  ControlMode mode=ControlMode::Manual;
  StopReason reason=StopReason::None;
  SafetyRequest safetyRequest=SafetyRequest::None;
  uint16_t flags=0;
  uint8_t activeWaypoint=0,enabledMask=0;
  bool saturated=false,physicalGate=false,waypointReached=false;
};

class Controller {
 public:
  explicit Controller(const Config& config=Config{});
  void reset();
  void setConfig(const Config& config);
  CommandResult setMode(ControlMode mode,uint32_t requestId,AuthoritativeSafety safety);
  CommandResult setManual(const ManualCommand& command,uint64_t receivedUs);
  CommandResult setHeading(float yawRad,uint32_t requestId);
  CommandResult setWaypoints(const Waypoint* points,uint8_t count,uint32_t requestId,AuthoritativeSafety safety);
  CommandResult setWaypointReachRadius(float radiusM,AuthoritativeSafety safety);
  Output step(const SensorInput& input);
  ControlMode mode() const { return mode_; }
  uint64_t manualReceivedUs() const { return manualReceivedUs_; }
  uint8_t manualOutputMask() const { return manual_.enabledMask; }
  uint8_t waypointCount() const { return waypointCount_; }
  uint8_t activeWaypoint() const { return activeWaypoint_; }
  static bool physicalConfigurationValid(const PhysicalConfig& config);

 private:
  bool fresh(bool valid,uint64_t timestamp,uint64_t now,uint32_t limit) const;
  bool finite(float value) const;
  float limit(float value,float previous,bool& saturated) const;
  void safe(Output& output,StopReason reason,SafetyRequest request) const;
  float speedBlend(float speed) const;
  float waypointCourse(const SensorInput& input,Output& output);

  Config config_{};
  ControlMode mode_=ControlMode::Manual;
  ManualCommand manual_{};
  uint64_t manualReceivedUs_=0,stallStartUs_=0;
  float targetYaw_=0;
  bool headingSet_=false,segmentStartSet_=false;
  Waypoint waypoints_[16]{};
  uint8_t waypointCount_=0,activeWaypoint_=0;
  float segmentStartNorth_=0,segmentStartEast_=0;
  float previous_[4]{};
};

const char* safetyName(AuthoritativeSafety value);
const char* modeName(ControlMode value);
const char* reasonName(StopReason value);

}  // namespace production_control
