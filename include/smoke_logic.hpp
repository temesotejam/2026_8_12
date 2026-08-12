#pragma once

#include <cstdint>

namespace smoke {

struct HeartbeatResult {
  std::uint32_t count;
  bool just_passed;
  bool passed;
};

class HeartbeatGate {
 public:
  explicit HeartbeatGate(std::uint32_t required_count = 3)
      : required_count_(required_count == 0 ? 1 : required_count) {}

  HeartbeatResult tick() {
    if (!passed_) {
      ++count_;
      if (count_ >= required_count_) {
        passed_ = true;
        return {count_, true, true};
      }
    }
    return {count_, false, passed_};
  }

  std::uint32_t count() const { return count_; }
  bool passed() const { return passed_; }

 private:
  std::uint32_t required_count_;
  std::uint32_t count_ = 0;
  bool passed_ = false;
};

}  // namespace smoke
