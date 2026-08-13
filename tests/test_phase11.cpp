#include "BoatLog2Replay.h"

#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace cores3sim::replay2;
using namespace production_control;
namespace fs=std::filesystem;

namespace {
void require(bool condition,const char* message){if(!condition){std::cerr<<"FAIL: "<<message<<'\n';std::exit(1);}}
SensorInput fresh(std::uint64_t now){SensorInput s{};s.heartbeat=s.imuValid=s.tofValid=s.gnssValid=s.powerValid=s.vescValid=true;s.safety=AuthoritativeSafety::Running;s.nowUs=now;s.heartbeatUs=s.imuUs=s.tofUs=s.gnssUs=s.powerUs=s.vescUs=now-1000;s.rollRad=.02f;s.pitchRad=-.01f;s.yawRad=.1f;s.rollRateRadS=.01f;s.pitchRateRadS=-.02f;s.yawRateRadS=.03f;s.tofM=.44f;s.groundSpeedMps=2.0f;s.busVoltageV=12.0f;s.currentA=2.0f;s.powerW=24.0f;s.vescErpm=1400;return s;}
void fixCrc(BoatLog2RecordRaw& r){r.record_crc32=recordCrc(r);}
}

int main(){
  static_assert(sizeof(BoatLog2HeaderRaw)==32,"header ABI");
  static_assert(sizeof(BoatLog2RecordRaw)==320,"record ABI");
  static_assert(sizeof(StepWire)<=300,"step payload fit");
  static_assert(sizeof(ConfigWire)<=300,"config payload fit");

  Config config{};config.kpPitch=1.1f;config.slewPerStep=.12f;config.physical.leftCenterUs=1510;
  const auto encodedConfig=encodeConfig(config);const auto decodedConfig=decodeConfig(encodedConfig);
  require(std::fabs(decodedConfig.kpPitch-config.kpPitch)<1e-7f,"config kp round trip");
  require(decodedConfig.physical.leftCenterUs==1510,"physical config round trip");

  auto sensor=fresh(1000000);sensor.northM=3.0f;sensor.eastM=-2.0f;
  const auto sensorRound=decodeSensorInput(encodeSensorInput(sensor));
  require(sensorRound.heartbeat&&sensorRound.safety==AuthoritativeSafety::Running,"sensor flags round trip");
  require(sensorRound.northM==3.0f&&sensorRound.eastM==-2.0f,"sensor numeric round trip");

  Controller controller{};std::vector<BoatLog2RecordRaw> records;std::uint32_t seq=1;
  controller.reset();records.push_back(makeResetRecord(seq++,0));
  controller.setConfig(config);records.push_back(makeConfigRecord(seq++,10,config));
  const auto modeResult=controller.setMode(ControlMode::Manual,1,AuthoritativeSafety::Disarmed);
  records.push_back(makeModeRecord(seq++,20,ControlMode::Manual,1,AuthoritativeSafety::Disarmed,modeResult));
  ManualCommand command{.2f,-.1f,.05f,.3f,ManualAll};const auto manualResult=controller.setManual(command,900000);
  records.push_back(makeManualRecord(seq++,30,command,900000,manualResult));
  auto input=fresh(1000000);const auto output=controller.step(input);records.push_back(makeStepRecord(seq++,input.nowUs,input,output));

  BoatLog2HeaderRaw header{};header.boot_id=123;header.start_us=500000;
  const fs::path root=fs::temp_directory_path()/"phase11_boatlog2_test";fs::remove_all(root);fs::create_directories(root);
  std::string error;
  require(writeBoatLog2(root/"good.bin",header,records,error),"write good log");
  BoatLog2File loaded{};require(readBoatLog2(root/"good.bin",loaded,error),"read good log");
  auto good=replay(loaded);require(good.pass,"deterministic replay must pass");
  require(good.processed_records==records.size()&&good.max_float_error<=kDefaultFloatEpsilon,"good replay metrics");

  auto outputMismatch=records;StepWire step{};std::memcpy(&step,outputMismatch.back().payload,sizeof(step));step.expected.right_front+=.04f;std::memcpy(outputMismatch.back().payload,&step,sizeof(step));fixCrc(outputMismatch.back());
  require(writeBoatLog2(root/"step_bad.bin",header,outputMismatch,error),"write step mismatch");
  require(readBoatLog2(root/"step_bad.bin",loaded,error),"read step mismatch");
  const auto stepBad=replay(loaded);require(!stepBad.pass&&stepBad.step_mismatches==1,"step mismatch detected");
  require(stepBad.first_failure_field.find("rightFront")!=std::string::npos,"step mismatch field");

  auto commandMismatch=records;SetModeWire mode{};std::memcpy(&mode,commandMismatch[2].payload,sizeof(mode));mode.expected.ack=static_cast<std::uint8_t>(Ack::Rejected);mode.expected.reason=77;std::memcpy(commandMismatch[2].payload,&mode,sizeof(mode));fixCrc(commandMismatch[2]);
  require(writeBoatLog2(root/"command_bad.bin",header,commandMismatch,error),"write command mismatch");
  require(readBoatLog2(root/"command_bad.bin",loaded,error),"read command mismatch");
  const auto cmdBad=replay(loaded);require(!cmdBad.pass&&cmdBad.command_mismatches==1,"command mismatch detected");

  auto crcBad=records;crcBad.back().payload[0]^=1;
  require(writeBoatLog2(root/"crc_bad.bin",header,crcBad,error),"write crc bad");
  require(readBoatLog2(root/"crc_bad.bin",loaded,error),"read crc bad");
  const auto crcReport=replay(loaded);require(!crcReport.pass&&crcReport.crc_failures==1,"CRC corruption detected");

  auto sequenceBad=records;sequenceBad[2].sequence=10;fixCrc(sequenceBad[2]);
  require(writeBoatLog2(root/"sequence_bad.bin",header,sequenceBad,error),"write sequence bad");
  require(readBoatLog2(root/"sequence_bad.bin",loaded,error),"read sequence bad");
  const auto seqReport=replay(loaded);require(!seqReport.pass&&seqReport.sequence_failures==1,"sequence gap detected");

  require(writeBoatLog2(root/"partial.bin",header,records,error),"write partial base");
  {std::ofstream out(root/"partial.bin",std::ios::binary|std::ios::app);char tail[37]{};out.write(tail,sizeof(tail));}
  require(readBoatLog2(root/"partial.bin",loaded,error),"read partial tail");
  const auto partial=replay(loaded);require(partial.pass&&partial.trailing_bytes==37,"partial final record tolerated");

  auto badHeader=header;badHeader.controller_abi_id^=1;require(writeBoatLog2(root/"bad_abi.bin",badHeader,records,error),"write bad ABI");
  require(!readBoatLog2(root/"bad_abi.bin",loaded,error)&&error.find("ABI")!=std::string::npos,"wrong controller ABI rejected");

  require(writeReplayCsv(root/"replay.csv",good,error),"write replay csv");
  require(writeReplayJson(root/"summary.json",good,kDefaultFloatEpsilon,error),"write replay json");
  require(writeReplaySvg(root/"summary.svg",good,kDefaultFloatEpsilon,error),"write replay svg");
  require(fs::file_size(root/"replay.csv")>40&&fs::file_size(root/"summary.json")>40&&fs::file_size(root/"summary.svg")>100,"report artifacts nonempty");

  std::cout<<"All phase-11 BOATLOG2 deterministic replay tests passed.\n";
  return 0;
}
