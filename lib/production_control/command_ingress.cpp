#include "command_ingress.h"
#include <cmath>
#include <cstring>
namespace production_control {
namespace {
bool validManual(const boat::ManualCommandPayload& c) {
  return std::isfinite(c.leftFrontWing) && std::isfinite(c.rightFrontWing) && std::isfinite(c.rearYaw) && std::isfinite(c.propulsion) && c.leftFrontWing >= -1 && c.leftFrontWing <= 1 && c.rightFrontWing >= -1 && c.rightFrontWing <= 1 && c.rearYaw >= -1 && c.rearYaw <= 1 && c.propulsion >= 0 && c.propulsion <= 1 && !(c.reserved[0]&~boat::ManualAll) && c.reserved[1]==0 && c.reserved[2]==0;
}
CommandIngressResult malformed(uint8_t type, uint64_t received, CommandReason reason, CommandIngressMetrics& metrics) {
  ++metrics.malformed; ++metrics.ackGenerated; CommandIngressResult r{}; r.recognized=true; r.ackGenerated=true; r.malformed=true; r.commandType=type; r.receivedUs=received; r.result={Ack::Rejected,(uint16_t)reason}; return r;
}
void save(CommandReplayWindow& replay, const CommandIngressResult& result, uint8_t version, uint16_t length, uint32_t fingerprint) {
  replay.store(result.requestId,result.commandSequence,result.commandType,version,length,fingerprint,(uint8_t)result.result.ack,result.result.reason,result.appliedUs);
}
}
CommandIngressResult CommandIngress::process(const boat::Frame& frame, AuthoritativeSafety safety, Controller& controller, uint64_t receivedUs) {
  const uint8_t type=frame.header.type; CommandIngressResult out{}; out.commandType=type; out.receivedUs=receivedUs; uint32_t fingerprint=0; uint8_t version=0; const uint16_t length=frame.header.length;
  if((boat::Type)type==boat::Type::ControlModeCommand) {
    if(length!=sizeof(boat::ControlModeCommandPayload)) return malformed(type,receivedUs,CommandReason::Type,metrics_);
    boat::ControlModeCommandPayload c{}; memcpy(&c,frame.payload,sizeof(c)); out.requestId=c.requestId; out.commandSequence=c.commandSequence; version=c.protocolVersion;
    if(c.protocolVersion!=boat::kVersion) return malformed(type,receivedUs,CommandReason::ProtocolVersion,metrics_);
    if(c.canonicalCrc!=boat::canonicalCrc(&c,offsetof(boat::ControlModeCommandPayload,canonicalCrc))) return malformed(type,receivedUs,CommandReason::CanonicalCrc,metrics_);
    if(c.mode>(uint8_t)ControlMode::WaypointOnly) return malformed(type,receivedUs,CommandReason::Range,metrics_);
    fingerprint=c.canonicalCrc;
  } else if((boat::Type)type==boat::Type::ManualCommand) {
    if(length!=sizeof(boat::ManualCommandPayload)) return malformed(type,receivedUs,CommandReason::Type,metrics_);
    boat::ManualCommandPayload c{}; memcpy(&c,frame.payload,sizeof(c)); out.requestId=c.requestId; out.commandSequence=c.commandSequence; version=c.protocolVersion;
    if(c.protocolVersion!=boat::kVersion) return malformed(type,receivedUs,CommandReason::ProtocolVersion,metrics_);
    if(c.canonicalCrc!=boat::canonicalCrc(&c,offsetof(boat::ManualCommandPayload,canonicalCrc))) return malformed(type,receivedUs,CommandReason::CanonicalCrc,metrics_);
    if(!validManual(c)) return malformed(type,receivedUs,CommandReason::Range,metrics_);
    fingerprint=c.canonicalCrc;
  } else if((boat::Type)type==boat::Type::HeadingTarget) {
    if(length!=sizeof(boat::HeadingTargetPayload)) return malformed(type,receivedUs,CommandReason::Type,metrics_);
    boat::HeadingTargetPayload c{}; memcpy(&c,frame.payload,sizeof(c)); out.requestId=c.requestId; out.commandSequence=c.commandSequence; version=c.protocolVersion;
    if(c.protocolVersion!=boat::kVersion) return malformed(type,receivedUs,CommandReason::ProtocolVersion,metrics_);
    if(c.canonicalCrc!=boat::canonicalCrc(&c,offsetof(boat::HeadingTargetPayload,canonicalCrc))) return malformed(type,receivedUs,CommandReason::CanonicalCrc,metrics_);
    if(!std::isfinite(c.targetYawRad)) return malformed(type,receivedUs,CommandReason::Range,metrics_);
    fingerprint=c.canonicalCrc;
  } else return out;
  out.recognized=true; const ReplayEntry* prior=nullptr; out.decision=replay_.inspect(out.requestId,out.commandSequence,type,version,length,fingerprint,prior);
  if(out.decision==ReplayDecision::Duplicate) { ++metrics_.duplicates; out.result={Ack::Duplicate,prior->reason}; out.original={(Ack)prior->disposition,prior->reason}; out.appliedUs=prior->appliedUs; out.ackGenerated=true; ++metrics_.ackGenerated; return out; }
  if(out.decision==ReplayDecision::ProtocolConflict) { ++metrics_.protocolConflicts; out.result={Ack::Conflict,(uint16_t)CommandReason::ReplayConflict}; out.ackGenerated=true; ++metrics_.ackGenerated; return out; }
  if(out.decision==ReplayDecision::Stale) { ++metrics_.stale; const bool ambiguous=replay_.hasHighWatermark() && CommandReplayWindow::ambiguous(out.commandSequence,replay_.highWatermark()); if(ambiguous) ++metrics_.ambiguousSequences; out.result={Ack::Stale,(uint16_t)(ambiguous?CommandReason::AmbiguousSequence:CommandReason::ReplayStale)}; out.ackGenerated=true; ++metrics_.ackGenerated; return out; }
  ++metrics_.newCommands;
  if((boat::Type)type==boat::Type::ControlModeCommand) { boat::ControlModeCommandPayload c{}; memcpy(&c,frame.payload,sizeof(c)); out.result=controller.setMode((ControlMode)c.mode,c.requestId,safety); }
  else if((boat::Type)type==boat::Type::ManualCommand) { boat::ManualCommandPayload c{}; memcpy(&c,frame.payload,sizeof(c)); out.result=controller.setManual({c.leftFrontWing,c.rightFrontWing,c.rearYaw,c.propulsion,c.reserved[0]},receivedUs); }
  else { boat::HeadingTargetPayload c{}; memcpy(&c,frame.payload,sizeof(c)); out.result=controller.setHeading(c.targetYawRad,c.requestId); }
  if(out.result.ack==Ack::Accepted) { ++metrics_.appliedCommands; out.appliedUs=receivedUs; } else ++metrics_.rejectedCommands;
  save(replay_,out,version,length,fingerprint); out.ackGenerated=true; ++metrics_.ackGenerated; return out;
}
} // namespace production_control
