#include "production_control.h"

#include <cmath>

namespace production_control {
namespace {
constexpr float kPi=3.14159265358979323846f;
float clampf(float value,float low,float high){return value<low?low:(value>high?high:value);}
float wrap(float value){while(value>kPi)value-=2*kPi;while(value<-kPi)value+=2*kPi;return value;}
}

Controller::Controller(const Config& config):config_(config){reset();}
void Controller::setConfig(const Config& config){config_=config;reset();}
void Controller::reset(){
  mode_=ControlMode::Manual;manual_={};manualReceivedUs_=0;stallStartUs_=0;
  targetYaw_=0;headingSet_=false;segmentStartSet_=false;
  waypointCount_=activeWaypoint_=0;segmentStartNorth_=segmentStartEast_=0;
  for(float& value:previous_)value=0;
}
bool Controller::finite(float value)const{return std::isfinite(value);}
bool Controller::fresh(bool valid,uint64_t timestamp,uint64_t now,uint32_t maxAge)const{
  return valid&&timestamp&&now>=timestamp&&now-timestamp<=maxAge;
}
CommandResult Controller::setMode(ControlMode mode,uint32_t,AuthoritativeSafety safety){
  if(safety!=AuthoritativeSafety::Disarmed)return{Ack::Rejected,1};
  if(static_cast<uint8_t>(mode)>static_cast<uint8_t>(ControlMode::WaypointOnly))return{Ack::Rejected,2};
  mode_=mode;headingSet_=false;stallStartUs_=0;
  return{Ack::Accepted,0};
}
CommandResult Controller::setManual(const ManualCommand& command,uint64_t receivedUs){
  if(!finite(command.leftFront)||!finite(command.rightFront)||!finite(command.rearYaw)||!finite(command.propulsion))return{Ack::Rejected,3};
  if(command.leftFront<-1||command.leftFront>1||command.rightFront<-1||command.rightFront>1||command.rearYaw<-1||command.rearYaw>1||command.propulsion<0||command.propulsion>1)return{Ack::Rejected,4};
  if(command.enabledMask&~ManualAll)return{Ack::Rejected,4};
  manual_=command;manualReceivedUs_=receivedUs;return{Ack::Accepted,0};
}
CommandResult Controller::setHeading(float yawRad,uint32_t){
  if(!finite(yawRad))return{Ack::Rejected,3};
  targetYaw_=wrap(yawRad);headingSet_=true;return{Ack::Accepted,0};
}
CommandResult Controller::setWaypoints(const Waypoint* points,uint8_t count,uint32_t,AuthoritativeSafety safety){
  if(safety!=AuthoritativeSafety::Disarmed||!points||!count||count>16)return{Ack::Rejected,5};
  for(uint8_t i=0;i<count;++i)if(!finite(points[i].northM)||!finite(points[i].eastM))return{Ack::Rejected,3};
  for(uint8_t i=0;i<count;++i)waypoints_[i]=points[i];
  waypointCount_=count;activeWaypoint_=0;segmentStartSet_=false;return{Ack::Accepted,0};
}
CommandResult Controller::setWaypointReachRadius(float radiusM,AuthoritativeSafety safety){
  if(safety!=AuthoritativeSafety::Disarmed||!finite(radiusM)||radiusM<.5f||radiusM>20.0f)return{Ack::Rejected,6};
  config_.waypointReachM=radiusM;return{Ack::Accepted,0};
}
bool Controller::physicalConfigurationValid(const PhysicalConfig& config){
  const uint8_t channels[]={config.leftChannel,config.rightChannel,config.rearChannel,config.propulsionChannel};
  for(uint8_t i=0;i<4;++i){if(channels[i]>15)return false;for(uint8_t j=i+1;j<4;++j)if(channels[i]==channels[j])return false;}
  return config.leftMinUs<config.leftCenterUs&&config.leftCenterUs<config.leftMaxUs&&
         config.rightMinUs<config.rightCenterUs&&config.rightCenterUs<config.rightMaxUs&&
         config.rearMinUs<config.rearCenterUs&&config.rearCenterUs<config.rearMaxUs&&
         config.propMinUs<config.propStopUs&&config.propStopUs<config.propMaxUs&&config.calibrationComplete;
}
float Controller::limit(float value,float previous,bool& saturated)const{
  const float bounded=clampf(value,-1.0f,1.0f);if(bounded!=value)saturated=true;value=bounded;
  const float delta=value-previous;
  if(delta>config_.slewPerStep){value=previous+config_.slewPerStep;saturated=true;}
  if(delta<-config_.slewPerStep){value=previous-config_.slewPerStep;saturated=true;}
  return value;
}
void Controller::safe(Output& output,StopReason reason,SafetyRequest request)const{
  output.leftFront=output.rightFront=output.rearYaw=output.propulsion=0;
  output.reason=reason;output.safetyRequest=request;output.physicalGate=false;
}
float Controller::speedBlend(float speed)const{
  return clampf((speed-config_.lowSpeedMps)/(config_.highSpeedMps-config_.lowSpeedMps),0.0f,1.0f);
}
float Controller::waypointCourse(const SensorInput& input,Output& output){
  if(!segmentStartSet_){segmentStartNorth_=input.northM;segmentStartEast_=input.eastM;segmentStartSet_=true;}
  const Waypoint& target=waypoints_[activeWaypoint_];
  const float dn=target.northM-input.northM,de=target.eastM-input.eastM;
  output.waypointDistanceM=std::sqrt(dn*dn+de*de);
  const float pathN=target.northM-segmentStartNorth_,pathE=target.eastM-segmentStartEast_;
  const float pathLength=std::sqrt(pathN*pathN+pathE*pathE);
  if(pathLength<.01f)return std::atan2(de,dn);
  const float pathCourse=std::atan2(pathE,pathN);
  const float relN=input.northM-segmentStartNorth_,relE=input.eastM-segmentStartEast_;
  const float crossTrack=-std::sin(pathCourse)*relN+std::cos(pathCourse)*relE;
  return wrap(pathCourse-std::atan2(crossTrack,config_.losLookaheadM));
}

Output Controller::step(const SensorInput& input){
  Output output{};output.safety=input.safety;output.mode=mode_;output.activeWaypoint=activeWaypoint_;
  if(input.safety!=AuthoritativeSafety::Running){safe(output,input.safety==AuthoritativeSafety::EStop?StopReason::EStop:StopReason::None,SafetyRequest::None);return output;}
  if(!fresh(input.heartbeat,input.heartbeatUs,input.nowUs,config_.heartbeatStaleUs)){safe(output,StopReason::Heartbeat,SafetyRequest::Fault);return output;}
  const bool manualMode=mode_==ControlMode::Manual;
  const bool waypointMode=modeUsesWaypoint(mode_);
  const bool attitudeControl=modeUsesAttitude(mode_);
  const bool needsManual=modeNeedsManual(mode_);
  if(needsManual&&!fresh(true,manualReceivedUs_,input.nowUs,config_.manualStaleUs)){safe(output,StopReason::ManualTimeout,SafetyRequest::Disarm);return output;}
  if(!manualMode&&!fresh(input.imuValid,input.imuUs,input.nowUs,config_.imuStaleUs)){safe(output,input.imuValid?StopReason::ImuStale:StopReason::ImuInvalid,SafetyRequest::Fault);return output;}
  if(waypointMode&&(!fresh(input.gnssValid,input.gnssUs,input.nowUs,config_.gnssStaleUs)||!waypointCount_)){safe(output,input.gnssValid?StopReason::GnssStale:StopReason::GnssInvalid,SafetyRequest::Fault);return output;}
  if(config_.enableAttitudeDangerTrip&&input.imuValid&&
     (std::fabs(input.pitchRad)>=config_.attitudeStopRad||
      std::fabs(input.rollRad)>=config_.attitudeStopRad)){
    safe(output,StopReason::AttitudeDanger,SafetyRequest::Fault);return output;
  }

  if(manualMode){
    output.enabledMask=manual_.enabledMask;
    output.leftPrelimit=(manual_.enabledMask&ManualLeft)?manual_.leftFront:0;
    output.rightPrelimit=(manual_.enabledMask&ManualRight)?manual_.rightFront:0;
    output.rearPrelimit=(manual_.enabledMask&ManualRear)?manual_.rearYaw:0;
    output.propulsionPrelimit=(manual_.enabledMask&ManualPropulsion)?manual_.propulsion:0;
  }else{
    output.enabledMask=ManualAll;
    if(attitudeControl){
      output.uPitch=config_.kpPitch*(config_.targetPitch-input.pitchRad)-config_.kdPitch*input.pitchRateRadS;
      output.uRoll=config_.kpRoll*(config_.targetRoll-input.rollRad)-config_.kdRoll*input.rollRateRadS;
      const bool tofFresh=fresh(input.tofValid,input.tofUs,input.nowUs,config_.tofStaleUs);
      if(tofFresh)output.uHeight=config_.kpHeight*(config_.targetHeightM-input.tofM);
      else{output.uHeight=0;output.flags|=HeightDegraded;output.reason=input.tofValid?StopReason::TofStale:StopReason::TofInvalid;}
      if(std::fabs(input.pitchRad)>=config_.pitchPriorityRad){
        output.uPitch=clampf(output.uPitch*1.5f,-1.0f,1.0f);output.uHeight=0;output.uRoll*=.25f;output.flags|=PitchPriority;
      }
      const float common=output.uPitch+output.uHeight;
      output.leftPrelimit=clampf(common+output.uRoll,-config_.attitudeServoLimit,config_.attitudeServoLimit);
      output.rightPrelimit=clampf(common-output.uRoll,-config_.attitudeServoLimit,config_.attitudeServoLimit);
    }else{
      output.leftPrelimit=0;output.rightPrelimit=0;
    }
    if(waypointMode){
      output.targetYaw=waypointCourse(input,output);
      if(output.waypointDistanceM<=config_.waypointReachM){
        output.waypointReached=true;
        segmentStartNorth_=waypoints_[activeWaypoint_].northM;segmentStartEast_=waypoints_[activeWaypoint_].eastM;
        if(++activeWaypoint_>=waypointCount_){safe(output,StopReason::FinalWaypoint,SafetyRequest::Disarm);return output;}
        output.activeWaypoint=activeWaypoint_;output.targetYaw=waypointCourse(input,output);
      }
    }else if(mode_==ControlMode::HeadingHold){
      if(!headingSet_){targetYaw_=wrap(input.yawRad);headingSet_=true;}
      output.targetYaw=targetYaw_;
    }
    if(mode_==ControlMode::HeadingHold||waypointMode){
      output.courseErrorRad=wrap(output.targetYaw-input.yawRad);
      const float blend=speedBlend(input.groundSpeedMps);
      const float gain=1.0f+blend*(config_.highSpeedYawGain-1.0f);
      const float yawLimit=1.0f+blend*(config_.highSpeedYawLimit-1.0f);
      output.uYaw=clampf(gain*config_.kpYaw*output.courseErrorRad-config_.kdYaw*input.yawRateRadS,-yawLimit,yawLimit);
      output.rearPrelimit=output.uYaw;output.flags|=YawScheduled;
    }else output.rearPrelimit=manual_.rearYaw;
    output.propulsionPrelimit=waypointMode?config_.autoPropulsion:manual_.propulsion;
  }

  if(!finite(output.leftPrelimit)||!finite(output.rightPrelimit)||!finite(output.rearPrelimit)||!finite(output.propulsionPrelimit)){safe(output,StopReason::NonFinite,SafetyRequest::Fault);return output;}
  if(output.propulsionPrelimit>.02f){
    const bool vescFresh=fresh(input.vescValid,input.vescUs,input.nowUs,config_.vescStaleUs);
    const bool powerFresh=fresh(input.powerValid,input.powerUs,input.nowUs,config_.powerStaleUs);
    if(input.vescFault||!vescFresh||!powerFresh){
      const StopReason unavailable=input.vescFault?StopReason::VescFault:(!vescFresh?StopReason::VescStale:(input.powerValid?StopReason::PowerStale:StopReason::PowerInvalid));
      if(waypointMode){safe(output,unavailable,SafetyRequest::Fault);return output;}
      output.propulsionPrelimit=0;output.flags|=PropulsionUnavailable;output.reason=unavailable;
    }
  }
  if(output.propulsionPrelimit>.02f){
    if(input.busVoltageV<=config_.criticalVoltageV){safe(output,StopReason::LowVoltage,SafetyRequest::Fault);return output;}
    if(std::fabs(input.currentA)>=config_.criticalCurrentA){safe(output,StopReason::OverCurrent,SafetyRequest::Fault);return output;}
    if(input.busVoltageV<config_.lowVoltageV){output.throttleLimit=.50f;output.flags|=VoltageLimited;output.reason=StopReason::LowVoltage;}
    if(std::fabs(input.currentA)>config_.overCurrentA){output.throttleLimit=clampf(output.throttleLimit,0.0f,.30f);output.flags|=CurrentLimited;output.reason=StopReason::OverCurrent;}
    const bool stalled=output.propulsionPrelimit>=config_.stallCommand&&std::fabs(input.vescErpm)<config_.stallErpm&&std::fabs(input.currentA)>config_.stallCurrentA;
    if(stalled){if(!stallStartUs_)stallStartUs_=input.nowUs;if(input.nowUs-stallStartUs_>=config_.stallTripUs){safe(output,StopReason::MotorStall,SafetyRequest::Fault);return output;}}
    else stallStartUs_=0;
    const bool cavitating=std::fabs(input.vescErpm)>config_.cavitationErpm&&std::fabs(input.currentA)<config_.cavitationCurrentA&&input.groundSpeedMps<config_.cavitationSpeedMps;
    if(cavitating){output.throttleLimit=clampf(output.throttleLimit,0.0f,.50f);output.flags|=CavitationLimited;output.reason=StopReason::Cavitation;}
    if(output.flags&PitchPriority)output.throttleLimit=clampf(output.throttleLimit,0.0f,.25f);
  }else stallStartUs_=0;

  output.propulsionPrelimit=clampf(output.propulsionPrelimit,0.0f,output.throttleLimit);
  output.leftFront=clampf(output.leftPrelimit,-1.0f,1.0f);
  output.rightFront=clampf(output.rightPrelimit,-1.0f,1.0f);
  output.rearYaw=clampf(output.rearPrelimit,-1.0f,1.0f);
  if(output.leftFront!=output.leftPrelimit||output.rightFront!=output.rightPrelimit||output.rearYaw!=output.rearPrelimit)output.saturated=true;
  output.propulsion=limit(output.propulsionPrelimit,previous_[3],output.saturated);
  if(output.propulsion<0)output.propulsion=0;
  previous_[0]=output.leftFront;previous_[1]=output.rightFront;previous_[2]=output.rearYaw;previous_[3]=output.propulsion;
  output.physicalGate=true;
  return output;
}

const char* safetyName(AuthoritativeSafety value){switch(value){case AuthoritativeSafety::Boot:return"BOOT";case AuthoritativeSafety::Disarmed:return"DISARMED";case AuthoritativeSafety::ArmedIdle:return"ARMED_IDLE";case AuthoritativeSafety::Running:return"RUNNING";case AuthoritativeSafety::EStop:return"E_STOP";default:return"FAULT";}}
const char* modeName(ControlMode value){switch(value){case ControlMode::Manual:return"MANUAL";case ControlMode::AttitudeAssist:return"ATTITUDE_ASSIST";case ControlMode::HeadingHold:return"HEADING_HOLD";case ControlMode::AutoWaypoint:return"AUTO_WAYPOINT";default:return"WAYPOINT_ONLY";}}
const char* reasonName(StopReason value){switch(value){case StopReason::None:return"NONE";case StopReason::Stop:return"STOP";case StopReason::EStop:return"E_STOP";case StopReason::Heartbeat:return"HEARTBEAT";case StopReason::ManualTimeout:return"MANUAL_TIMEOUT";case StopReason::ImuInvalid:return"IMU_INVALID";case StopReason::ImuStale:return"IMU_STALE";case StopReason::TofInvalid:return"TOF_INVALID";case StopReason::TofStale:return"TOF_STALE";case StopReason::GnssInvalid:return"GNSS_INVALID";case StopReason::GnssStale:return"GNSS_STALE";case StopReason::VescFault:return"VESC_FAULT";case StopReason::NonFinite:return"NONFINITE";case StopReason::FinalWaypoint:return"FINAL_WAYPOINT";case StopReason::PowerInvalid:return"POWER_INVALID";case StopReason::PowerStale:return"POWER_STALE";case StopReason::LowVoltage:return"LOW_VOLTAGE";case StopReason::OverCurrent:return"OVER_CURRENT";case StopReason::MotorStall:return"MOTOR_STALL";case StopReason::AttitudeDanger:return"ATTITUDE_DANGER";case StopReason::VescStale:return"VESC_STALE";default:return"CAVITATION";}}

}  // namespace production_control
