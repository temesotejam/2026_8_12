#include "DualControllerSystem.h"
#include "VirtualDuplexUart.h"

#include <cstdlib>
#include <iostream>

using namespace cores3sim;

namespace {
void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}
}

int main() {
  {
    // 92160 bytes * 10 bits/byte at 921600 bit/s = exactly one second.
    VirtualDuplexUart link(921600, 200000);
    link.configure(UartDirection::CommToControl, true, 0);
    require(link.send(UartDirection::CommToControl,
                      0,
                      LinkPacketType::GnssNav,
                      1,
                      92160),
            "bandwidth packet queued");
    link.advance(999);
    require(link.takeForControl().empty(),
            "92160 bytes at 8N1 must not arrive before 1s");
    link.advance(1000);
    require(link.takeForControl().size() == 1,
            "921600 baud 8N1 must deliver 92160 bytes in 1s");
  }

  {
    VirtualDuplexUart link;
    link.configure(UartDirection::CommToControl, true, 0);

    link.injectNext(UartDirection::CommToControl,
                    InjectedUartFault::Corrupt);
    link.send(UartDirection::CommToControl,
              0,
              LinkPacketType::GnssNav,
              1,
              64);
    link.advance(10);
    require(link.takeForControl().empty(),
            "corrupt frame must be rejected");
    require(link.stats().comm_to_control.crc_errors == 1,
            "CRC counter");

    link.injectNext(UartDirection::CommToControl,
                    InjectedUartFault::Framing);
    link.send(UartDirection::CommToControl,
              20,
              LinkPacketType::GnssNav,
              2,
              64);
    link.advance(30);
    require(link.stats().comm_to_control.framing_errors == 1,
            "framing counter");

    link.injectNext(UartDirection::CommToControl,
                    InjectedUartFault::Drop);
    link.send(UartDirection::CommToControl,
              40,
              LinkPacketType::GnssNav,
              3,
              64);
    require(link.stats().comm_to_control.dropped_injected == 1,
            "injected drop counter");
  }

  DualControllerSystem system;
  DualControllerInput input;
  DualControllerStatus status;

  for (std::uint32_t t = 0; t <= 700; t += 10) {
    status = system.step(t, input);
  }
  require(status.control_running,
          "normal link keeps control running");
  require(status.control_sees_comm_link &&
              status.comm_sees_control_link,
          "both heartbeat directions healthy");
  require(status.gnss_received >= 6 &&
              status.results_received >= 6,
          "10 Hz data exchange active");

  // GNSS is queued before the heartbeat on a normal data tick, so the one-shot
  // corruption below rejects one GNSS packet without killing the heartbeat.
  input.comm_to_control_fault = InjectedUartFault::Corrupt;
  status = system.step(800, input);
  input.comm_to_control_fault = InjectedUartFault::None;
  for (std::uint32_t t = 810; t <= 850; t += 10) {
    status = system.step(t, input);
  }
  require(system.uartStats().comm_to_control.crc_errors >= 1,
          "corrupt GNSS frame counted");
  require(status.control_running,
          "single corrupt frame is not a stop condition");

  // A one-way communication-side outage is enough to expire the control-side
  // heartbeat and must force a local failsafe stop.
  input.comm_to_control_connected = false;
  for (std::uint32_t t = 900; t <= 1450; t += 10) {
    status = system.step(t, input);
  }
  require(status.control_failsafe_stop &&
              !status.control_running,
          "heartbeat timeout must failsafe-stop control");
  require(system.stats().failsafe_stops == 1,
          "failsafe stop counter");

  // STOP propagation is tested independently on a healthy link so it is not
  // hidden by the heartbeat-failsafe case above.
  DualControllerSystem stop_system;
  DualControllerInput stop_input;
  for (std::uint32_t t = 0; t <= 500; t += 10) {
    stop_system.step(t, stop_input);
  }
  stop_input.send_stop = true;
  stop_system.step(600, stop_input);
  stop_input.send_stop = false;

  DualControllerStatus stop_status;
  for (std::uint32_t t = 610; t <= 650; t += 10) {
    stop_status = stop_system.step(t, stop_input);
  }
  require(stop_status.stop_received == 1 &&
              !stop_status.control_running &&
              !stop_status.control_failsafe_stop,
          "STOP must propagate independently of failsafe");

  // Reverse-link loss must be visible on the communication side even though
  // the control side can still receive its heartbeat direction.
  DualControllerSystem reverse_system;
  DualControllerInput reverse_input;
  for (std::uint32_t t = 0; t <= 700; t += 10) {
    reverse_system.step(t, reverse_input);
  }
  reverse_input.control_to_comm_connected = false;
  DualControllerStatus reverse_status;
  for (std::uint32_t t = 800; t <= 1350; t += 10) {
    reverse_status = reverse_system.step(t, reverse_input);
  }
  require(!reverse_status.comm_sees_control_link,
          "communication side detects control heartbeat loss");

  std::cout << "All phase-7 dual-controller UART tests passed.\n";
  return 0;
}
