#pragma once
#include <stdint.h>
#include "boat_protocol.h"
#include "production_control.h"
#include "command_replay.h"
namespace production_control {
// Syntax failures are not replayed. State rejections are replayed so retry does not re-evaluate.
enum class CommandReason : uint16_t { None=0, Safety=1, CanonicalCrc=2, Type=3, Range=4, AmbiguousSequence=5, ProtocolVersion=6, ReplayConflict=7, ReplayStale=8 };
struct CommandIngressMetrics { uint32_t newCommands=0,appliedCommands=0,rejectedCommands=0,duplicates=0,protocolConflicts=0,stale=0,ambiguousSequences=0,malformed=0,ackGenerated=0,duplicateReapply=0; };
struct CommandIngressResult { bool recognized=false,ackGenerated=false,malformed=false; uint8_t commandType=0; uint32_t requestId=0,commandSequence=0; uint64_t receivedUs=0,appliedUs=0; CommandResult result{}; CommandResult original{}; ReplayDecision decision=ReplayDecision::New; };
class CommandIngress {
 public:
  CommandIngressResult process(const boat::Frame& frame,AuthoritativeSafety safety,Controller& controller,uint64_t receivedUs);
  const CommandIngressMetrics& metrics()const{return metrics_;} const CommandReplayWindow& replay()const{return replay_;}
  void reset(){replay_=CommandReplayWindow{};metrics_={};}
 private: CommandReplayWindow replay_{}; CommandIngressMetrics metrics_{};
};
} // namespace production_control
