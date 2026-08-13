#include "BoatLog2Replay.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace cores3sim::replay2;
using namespace production_control;

namespace {
SensorInput freshInput(std::uint64_t now_us, AuthoritativeSafety safety) {
  SensorInput s{};
  s.heartbeat=true; s.imuValid=true; s.tofValid=true; s.gnssValid=true;
  s.powerValid=true; s.vescValid=true; s.vescFault=false; s.safety=safety;
  s.nowUs=now_us; s.heartbeatUs=now_us-10000; s.imuUs=now_us-5000;
  s.tofUs=now_us-10000; s.gnssUs=now_us-20000; s.powerUs=now_us-10000; s.vescUs=now_us-10000;
  s.rollRad=0.03f; s.pitchRad=-0.02f; s.yawRad=0.10f;
  s.rollRateRadS=0.01f; s.pitchRateRadS=-0.015f; s.yawRateRadS=0.02f;
  s.tofM=0.42f; s.northM=0.0f; s.eastM=0.0f; s.groundSpeedMps=2.2f; s.courseRad=0.12f;
  s.busVoltageV=12.1f; s.currentA=2.5f; s.powerW=30.25f; s.vescErpm=1500.0f;
  return s;
}

void refreshCrc(BoatLog2RecordRaw& r) { r.record_crc32=recordCrc(r); }

StepWire stepPayload(const BoatLog2RecordRaw& r) {
  StepWire wire{};
  std::memcpy(&wire,r.payload,sizeof(wire));
  return wire;
}

SetModeWire modePayload(const BoatLog2RecordRaw& r) {
  SetModeWire wire{};
  std::memcpy(&wire,r.payload,sizeof(wire));
  return wire;
}

void overwritePayload(BoatLog2RecordRaw& r,const void* payload,std::size_t n) {
  std::memset(r.payload,0,sizeof(r.payload));
  std::memcpy(r.payload,payload,n);
  r.payload_bytes=static_cast<std::uint16_t>(n);
  refreshCrc(r);
}
}

int main(int argc,char** argv){
  const fs::path outdir=argc>1?argv[1]:"artifacts/boatlog2";
  fs::create_directories(outdir);
  std::string error;
  BoatLog2HeaderRaw header{}; header.boot_id=0xb0020011u; header.start_us=1000000ULL;
  std::vector<BoatLog2RecordRaw> records;
  Controller controller{};
  std::uint32_t seq=1;
  auto pushReset=[&](std::uint64_t at){controller.reset();records.push_back(makeResetRecord(seq++,at));};
  auto pushConfig=[&](std::uint64_t at,const Config& c){controller.setConfig(c);records.push_back(makeConfigRecord(seq++,at,c));};
  auto pushMode=[&](std::uint64_t at,ControlMode mode,std::uint32_t id,AuthoritativeSafety safety){const auto result=controller.setMode(mode,id,safety);records.push_back(makeModeRecord(seq++,at,mode,id,safety,result));};
  auto pushManual=[&](std::uint64_t at,const ManualCommand& command,std::uint64_t received){const auto result=controller.setManual(command,received);records.push_back(makeManualRecord(seq++,at,command,received,result));};
  auto pushHeading=[&](std::uint64_t at,float yaw,std::uint32_t id){const auto result=controller.setHeading(yaw,id);records.push_back(makeHeadingRecord(seq++,at,yaw,id,result));};
  auto pushReach=[&](std::uint64_t at,float radius,AuthoritativeSafety safety){const auto result=controller.setWaypointReachRadius(radius,safety);records.push_back(makeReachRecord(seq++,at,radius,safety,result));};
  auto pushWaypoints=[&](std::uint64_t at,const Waypoint* points,std::uint8_t count,std::uint32_t id,AuthoritativeSafety safety){const auto result=controller.setWaypoints(points,count,id,safety);records.push_back(makeWaypointsRecord(seq++,at,points,count,id,safety,result));};
  auto pushStep=[&](SensorInput input){const auto result=controller.step(input);records.push_back(makeStepRecord(seq++,input.nowUs,input,result));};

  Config config{}; config.slewPerStep=0.08f; config.enableAttitudeDangerTrip=true;
  pushReset(0); pushConfig(10,config);
  pushMode(20,ControlMode::Manual,1001,AuthoritativeSafety::Disarmed);
  ManualCommand manual{0.25f,-0.15f,0.10f,0.35f,ManualAll};
  pushManual(30,manual,1000000ULL);
  pushStep(freshInput(1050000ULL,AuthoritativeSafety::Running));
  auto staleManual=freshInput(1600001ULL,AuthoritativeSafety::Running); pushStep(staleManual);

  pushReset(1700000ULL); pushConfig(1700010ULL,config);
  pushMode(1700020ULL,ControlMode::HeadingHold,1002,AuthoritativeSafety::Disarmed);
  pushManual(1700030ULL,ManualCommand{0,0,0,0.20f,ManualPropulsion},2000000ULL);
  pushHeading(1700040ULL,0.70f,1003);
  pushStep(freshInput(2050000ULL,AuthoritativeSafety::Running));

  pushReset(2200000ULL); pushConfig(2200010ULL,config);
  pushReach(2200020ULL,0.10f,AuthoritativeSafety::Disarmed);  // Expected rejection.
  pushReach(2200030ULL,1.00f,AuthoritativeSafety::Disarmed);
  Waypoint points[2]{{10.0f,0.0f},{20.0f,5.0f}};
  pushWaypoints(2200040ULL,points,2,2001,AuthoritativeSafety::Disarmed);
  pushMode(2200050ULL,ControlMode::AutoWaypoint,2002,AuthoritativeSafety::Disarmed);
  pushStep(freshInput(3000000ULL,AuthoritativeSafety::Disarmed));
  auto nav1=freshInput(3100000ULL,AuthoritativeSafety::Running); nav1.northM=0.0f;nav1.eastM=0.0f;pushStep(nav1);
  auto nav2=freshInput(3200000ULL,AuthoritativeSafety::Running); nav2.northM=9.5f;nav2.eastM=0.0f;pushStep(nav2);
  auto degraded=freshInput(3300000ULL,AuthoritativeSafety::Running);degraded.northM=10.5f;degraded.eastM=0.3f;degraded.tofValid=false;pushStep(degraded);
  auto heartbeatFail=freshInput(4000000ULL,AuthoritativeSafety::Running);heartbeatFail.heartbeatUs=3000000ULL;heartbeatFail.northM=11.0f;pushStep(heartbeatFail);

  if(!writeBoatLog2(outdir/"deterministic.bin",header,records,error)) throw std::runtime_error(error);

  auto stepMismatch=records;
  for(auto& r:stepMismatch){if(r.type==static_cast<std::uint8_t>(RecordType::Step)){auto w=stepPayload(r);w.expected.left_front+=0.05f;overwritePayload(r,&w,sizeof(w));break;}}
  if(!writeBoatLog2(outdir/"step_mismatch.bin",header,stepMismatch,error)) throw std::runtime_error(error);

  auto commandMismatch=records;
  for(auto& r:commandMismatch){if(r.type==static_cast<std::uint8_t>(RecordType::SetMode)){auto w=modePayload(r);w.expected.ack=static_cast<std::uint8_t>(Ack::Rejected);w.expected.reason=99;overwritePayload(r,&w,sizeof(w));break;}}
  if(!writeBoatLog2(outdir/"command_mismatch.bin",header,commandMismatch,error)) throw std::runtime_error(error);

  auto crcCorrupt=records;
  for(auto& r:crcCorrupt){if(r.type==static_cast<std::uint8_t>(RecordType::Step)){r.payload[5]^=0x40;break;}}
  if(!writeBoatLog2(outdir/"crc_corrupt.bin",header,crcCorrupt,error)) throw std::runtime_error(error);

  auto sequenceBad=records;
  if(sequenceBad.size()>3){sequenceBad[3].sequence=99;refreshCrc(sequenceBad[3]);}
  if(!writeBoatLog2(outdir/"sequence_bad.bin",header,sequenceBad,error)) throw std::runtime_error(error);

  if(!writeBoatLog2(outdir/"partial_tail.bin",header,records,error)) throw std::runtime_error(error);
  {std::ofstream out(outdir/"partial_tail.bin",std::ios::binary|std::ios::app);std::uint8_t tail[37]{};for(std::size_t i=0;i<sizeof(tail);++i)tail[i]=static_cast<std::uint8_t>(i+1);out.write(reinterpret_cast<const char*>(tail),sizeof(tail));}

  std::cout<<"Generated "<<records.size()<<" deterministic BOATLOG2 records in "<<outdir<<"\n";
  return 0;
}
