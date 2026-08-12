#include "VirtualHardware.h"

#include <cstdlib>
#include <iostream>
#include <limits>

using cores3sim::VirtualHardware;
using cores3sim::VirtualHwInput;

namespace {
void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}
}

int main() {
  VirtualHardware hw;
  VirtualHwInput in;
  in.pitch_deg = 12.5f;
  in.i2c_latency_ms = 2;
  in.uart_latency_ms = 0;

  hw.apply(0, in);
  auto s = hw.sample(0);
  require(s.i2c_ok && s.imu_ok, "normal I2C IMU read must succeed");
  require(s.pitch_deg == 12.5f, "successful I2C read must update pitch");
  require(s.uart_ok && s.gnss_ok, "zero-latency UART frame must arrive");

  in.pitch_deg = 30.0f;
  in.i2c_latency_ms = VirtualHardware::kI2cTimeoutMs + 1;
  hw.apply(100, in);
  s = hw.sample(100);
  require(!s.i2c_ok && !s.imu_ok, "excess I2C latency must time out");
  require(s.pitch_deg == 12.5f, "failed I2C read must keep last good pitch");

  in.i2c_latency_ms = 1;
  in.i2c_connected = false;
  hw.apply(200, in);
  s = hw.sample(200);
  require(!s.i2c_ok, "disconnected I2C bus must fail");

  VirtualHardware uart;
  VirtualHwInput u;
  u.uart_latency_ms = 200;
  uart.apply(0, u);
  auto u0 = uart.sample(0);
  require(!u0.gnss_ok, "delayed UART frame must not arrive early");
  require(u0.gnss_age_ms == std::numeric_limits<std::uint32_t>::max(),
          "GNSS age must be unknown before first frame");

  u.gnss_source_valid = false;
  uart.apply(100, u);
  require(!uart.sample(100).gnss_ok, "GNSS must still be unavailable before delivery");
  uart.apply(200, u);
  auto u200 = uart.sample(200);
  require(u200.gnss_ok && u200.gnss_age_ms == 0,
          "UART frame must arrive after configured latency");

  u.uart_connected = false;
  uart.apply(300, u);
  auto u300 = uart.sample(300);
  require(!u300.uart_ok, "UART disconnect must be reported immediately");
  uart.apply(700, u);
  auto u700 = uart.sample(700);
  require(u700.gnss_ok, "GNSS remains recent through the stale threshold");
  uart.apply(701, u);
  auto u701 = uart.sample(701);
  require(!u701.gnss_ok, "GNSS must become stale after threshold");

  require(hw.stats().i2c_timeouts >= 2, "I2C timeout counter must track failures");
  require(uart.stats().uart_frames_delivered == 1,
          "UART delivery counter must track delivered frames");

  std::cout << "All virtual hardware tests passed.\n";
  return 0;
}
