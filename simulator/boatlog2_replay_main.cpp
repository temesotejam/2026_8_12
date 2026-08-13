#include "BoatLog2Replay.h"

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;
using namespace cores3sim::replay2;

int main(int argc,char** argv){
  if(argc<3){
    std::cerr<<"usage: boatlog2_replay_cli <BOATLOG2.bin> <output-dir> [float-epsilon]\n";
    return 64;
  }
  const fs::path source=argv[1];
  const fs::path outdir=argv[2];
  const float epsilon=argc>=4?std::stof(argv[3]):kDefaultFloatEpsilon;
  fs::create_directories(outdir);
  BoatLog2File log{};std::string error;
  if(!readBoatLog2(source,log,error)){std::cerr<<"BOATLOG2 read failed: "<<error<<'\n';return 1;}
  const ReplayReport report=replay(log,epsilon);
  if(!writeReplayCsv(outdir/"replay.csv",report,error))throw std::runtime_error(error);
  if(!writeReplayJson(outdir/"summary.json",report,epsilon,error))throw std::runtime_error(error);
  if(!writeReplaySvg(outdir/"summary.svg",report,epsilon,error))throw std::runtime_error(error);
  std::cout<<"BOATLOG2 replay "<<(report.pass?"PASS":"FAIL")
           <<" records="<<report.processed_records<<'/'<<report.total_records
           <<" trailing="<<report.trailing_bytes
           <<" command_mismatch="<<report.command_mismatches
           <<" step_mismatch="<<report.step_mismatches
           <<" crc="<<report.crc_failures
           <<" sequence="<<report.sequence_failures
           <<" max_float_error="<<report.max_float_error<<'\n';
  if(!report.pass&&report.first_failure_sequence){
    std::cout<<"first failure seq="<<report.first_failure_sequence
             <<" field="<<report.first_failure_field<<'\n';
  }
  return report.pass?0:2;
}
