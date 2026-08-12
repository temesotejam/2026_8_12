#include <cstdlib>
#include <iostream>

#include "smoke_logic.hpp"

int main() {
  smoke::HeartbeatGate gate{3};

  const auto first = gate.tick();
  if (first.count != 1 || first.passed || first.just_passed) {
    std::cerr << "NATIVE_CHECK:FAIL first-heartbeat\n";
    return EXIT_FAILURE;
  }

  const auto second = gate.tick();
  if (second.count != 2 || second.passed || second.just_passed) {
    std::cerr << "NATIVE_CHECK:FAIL second-heartbeat\n";
    return EXIT_FAILURE;
  }

  const auto third = gate.tick();
  if (third.count != 3 || !third.passed || !third.just_passed) {
    std::cerr << "NATIVE_CHECK:FAIL third-heartbeat\n";
    return EXIT_FAILURE;
  }

  const auto fourth = gate.tick();
  if (fourth.count != 3 || !fourth.passed || fourth.just_passed) {
    std::cerr << "NATIVE_CHECK:FAIL latched-pass\n";
    return EXIT_FAILURE;
  }

  std::cout << "NATIVE_CHECK:PASS\n";
  return EXIT_SUCCESS;
}
