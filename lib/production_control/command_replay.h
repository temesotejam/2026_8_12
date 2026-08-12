#pragma once
#include <stdint.h>
namespace production_control {
enum class ReplayDecision : uint8_t { New=0, Duplicate=1, ProtocolConflict=2, Stale=3 };
struct ReplayEntry { bool valid=false; uint32_t requestId=0,sequence=0,fingerprint=0; uint8_t type=0,version=0; uint16_t length=0,reason=0; uint8_t disposition=0; uint64_t appliedUs=0; };
class CommandReplayWindow {
 public:
  static constexpr uint8_t kCapacity=64;
  ReplayDecision inspect(uint32_t requestId,uint32_t sequence,uint8_t type,uint8_t version,uint16_t length,uint32_t fingerprint,const ReplayEntry*& match) const;
  void store(uint32_t requestId,uint32_t sequence,uint8_t type,uint8_t version,uint16_t length,uint32_t fingerprint,uint8_t disposition,uint16_t reason,uint64_t appliedUs);
  static bool newer(uint32_t candidate,uint32_t baseline);
  static bool ambiguous(uint32_t candidate,uint32_t baseline);
  uint32_t duplicateCount()const{return duplicateCount_;} uint32_t conflictCount()const{return conflictCount_;} uint32_t staleCount()const{return staleCount_;} uint32_t appliedCount()const{return appliedCount_;}
  bool hasHighWatermark()const{return hasHigh_;} uint32_t highWatermark()const{return high_;}
 private:
  ReplayEntry entries_[kCapacity]{}; uint8_t next_=0; bool hasHigh_=false; uint32_t high_=0; mutable uint32_t duplicateCount_=0,conflictCount_=0,staleCount_=0; uint32_t appliedCount_=0;
};
} // namespace production_control
