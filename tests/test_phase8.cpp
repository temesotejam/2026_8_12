#include "ProductionProtocolSystem.h"
#include "ProductionUartLink.h"
#include "boat_protocol.h"

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>

using namespace cores3sim;

namespace {
void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

std::uint32_t decoderErrors(const ProtocolDecoderStats& s) {
  return s.control_crc_errors + s.control_cobs_errors + s.control_length_errors +
         s.comm_crc_errors + s.comm_cobs_errors + s.comm_length_errors;
}

void runRange(ProductionProtocolSystem& system,
              ProductionProtocolInput& input,
              std::uint32_t from,
              std::uint32_t to,
              std::uint32_t step = 10) {
  for (std::uint32_t t = from; t <= to; t += step) {
    system.step(t, input);
  }
}
}

int main() {
  static_assert(sizeof(boat::Header) == 22, "production header ABI");
  static_assert(sizeof(boat::HeartbeatPayload) == 12, "heartbeat ABI");
  static_assert(sizeof(boat::CommandPayload) == 8, "command ABI");
  static_assert(sizeof(boat::CommandAckPayload) == 28, "command ACK ABI");
  static_assert(sizeof(boat::GnssNavV2Payload) == 68, "GNSS_NAV_V2 ABI");
  static_assert(static_cast<std::uint8_t>(boat::Type::Heartbeat) == 32,
                "heartbeat type ABI");
  static_assert(static_cast<std::uint8_t>(boat::Type::Stop) == 36,
                "STOP type ABI");
  static_assert(static_cast<std::uint8_t>(boat::Type::GnssNavV2) == 59,
                "GNSS_NAV_V2 type ABI");

  {
    // Exact production codec copied from the boat firmware: Header + payload +
    // CRC32, COBS encoded, then terminated by zero.
    boat::HeartbeatPayload heartbeat{1234, 77, 1, 0, 0};
    boat::Header header{boat::kVersion,
                        static_cast<std::uint8_t>(boat::Type::Heartbeat),
                        static_cast<std::uint16_t>(sizeof(heartbeat)),
                        9,
                        42,
                        123456,
                        0};
    std::uint8_t encoded[boat::kMaxEncoded]{};
    const std::size_t bytes = boat::encode(
        header,
        reinterpret_cast<const std::uint8_t*>(&heartbeat),
        encoded,
        sizeof(encoded));
    require(bytes > 0 && encoded[bytes - 1] == 0,
            "production frame must use COBS zero delimiter");

    boat::Decoder decoder;
    boat::Frame decoded{};
    bool complete = false;
    for (std::size_t i = 0; i < bytes; ++i) {
      complete = decoder.feed(encoded[i], decoded) || complete;
    }
    require(complete, "production heartbeat must decode");
    require(decoded.header.version == boat::kVersion &&
                decoded.header.sequence == 9 &&
                decoded.header.type ==
                    static_cast<std::uint8_t>(boat::Type::Heartbeat),
            "decoded production header must match");
    require(decoded.header.length == sizeof(heartbeat) &&
                std::memcmp(decoded.payload, &heartbeat, sizeof(heartbeat)) == 0,
            "decoded production heartbeat payload must match");

    // A changed encoded byte must be rejected by the same production decoder.
    std::uint8_t corrupt[boat::kMaxEncoded]{};
    std::memcpy(corrupt, encoded, bytes);
    std::size_t index = bytes / 2;
    if (index >= bytes - 1) index = bytes - 2;
    corrupt[index] ^= 0x01u;
    if (corrupt[index] == 0) corrupt[index] = 0x7fu;
    boat::Decoder reject_decoder;
    boat::Frame rejected{};
    bool rejected_complete = false;
    for (std::size_t i = 0; i < bytes; ++i) {
      rejected_complete = reject_decoder.feed(corrupt[i], rejected) ||
                          rejected_complete;
    }
    require(!rejected_complete,
            "corrupted production wire frame must not decode");
    require(reject_decoder.crcErrors + reject_decoder.cobsErrors +
                reject_decoder.lengthErrors >= 1,
            "production decoder must classify corruption");
  }

  {
    // Actual encoded bytes are serialized at the configured UART speed.
    boat::HeartbeatPayload heartbeat{1, 2, 1, 0, 0};
    boat::Header header{boat::kVersion,
                        static_cast<std::uint8_t>(boat::Type::Heartbeat),
                        static_cast<std::uint16_t>(sizeof(heartbeat)),
                        1, 2, 1000, 0};
    std::uint8_t encoded[boat::kMaxEncoded]{};
    const std::size_t bytes = boat::encode(
        header,
        reinterpret_cast<const std::uint8_t*>(&heartbeat),
        encoded,
        sizeof(encoded));
    ProductionUartLink link(921600, 4096);
    link.configure(UartDirection::CommToControl, true, 0);
    require(link.send(UartDirection::CommToControl, 0, encoded, bytes),
            "encoded heartbeat queued on production UART");
    link.advance(0);
    require(link.takeForControl().empty(),
            "non-empty 8N1 frame cannot arrive in zero time");
    link.advance(1);
    require(link.takeForControl().size() == 1,
            "small production frame should arrive within one ms at 921600");
  }

  {
    ProductionProtocolSystem system;
    ProductionProtocolInput input;
    ProductionProtocolStatus status{};

    runRange(system, input, 0, 290);
    status = system.step(290, input);
    require(status.comm_link_fresh,
            "communication node must see fresh real control frames");
    require(status.control_host_heartbeat_fresh,
            "control node must receive gated communication heartbeat");
    require(status.gnss_received >= 2,
            "control node must decode real GNSS_NAV_V2 frames");
    require(status.control_safety == ProductionSafety::Disarmed,
            "production control starts DISARMED");

    input.send_arm = true;
    system.step(300, input);
    input.send_arm = false;
    runRange(system, input, 310, 350);
    status = system.step(350, input);
    require(status.control_safety == ProductionSafety::Armed,
            "real ARM CommandPayload must arm control");
    require(status.arm_received == 1 && status.command_acks_received >= 1,
            "ARM must be received and ACKed through real protocol");

    input.send_start = true;
    system.step(400, input);
    input.send_start = false;
    runRange(system, input, 410, 450);
    status = system.step(450, input);
    require(status.control_safety == ProductionSafety::Running &&
                status.control_running,
            "real START command must enter RUNNING");

    const std::uint32_t errors_before = decoderErrors(system.decoderStats());
    input.comm_to_control_fault = InjectedUartFault::Corrupt;
    system.step(500, input);
    input.comm_to_control_fault = InjectedUartFault::None;
    runRange(system, input, 510, 560);
    status = system.step(560, input);
    require(decoderErrors(system.decoderStats()) > errors_before,
            "wire corruption must be rejected by production COBS/CRC decoder");
    require(status.control_safety == ProductionSafety::Running &&
                status.control_running,
            "one corrupt GNSS frame must not stop healthy heartbeat control");
    require(system.uartStats().comm_to_control.corruptions_injected >= 1,
            "wire corruption injection counter");

    // Break only CONTROL->COMM. The communication node initially still sends
    // heartbeat because its last real control frame is fresh. After 1s it stops
    // heartbeat; 500ms later the control node must self-stop.
    input.control_to_comm_connected = false;
    runRange(system, input, 700, 2200);
    status = system.step(2200, input);
    require(!status.comm_link_fresh,
            "communication node must mark reverse link stale");
    require(status.control_failsafe_stop && !status.control_running,
            "bidirectional heartbeat policy must cause local failsafe stop");
    require(status.control_safety == ProductionSafety::Disarmed &&
                status.stop_reason == ProductionStopReason::Heartbeat,
            "heartbeat failsafe must return control to DISARMED");
    require(system.stats().failsafe_stops == 1,
            "failsafe must latch once");
  }

  {
    // STOP is tested independently on a healthy protocol link.
    ProductionProtocolSystem system;
    ProductionProtocolInput input;
    runRange(system, input, 0, 290);
    input.send_arm = true;
    system.step(300, input);
    input.send_arm = false;
    runRange(system, input, 310, 390);
    input.send_start = true;
    system.step(400, input);
    input.send_start = false;
    runRange(system, input, 410, 490);
    auto status = system.step(490, input);
    require(status.control_running, "STOP test must reach RUNNING first");

    input.send_stop = true;
    system.step(500, input);
    input.send_stop = false;
    runRange(system, input, 510, 560);
    status = system.step(560, input);
    require(status.stop_received == 1 &&
                status.control_safety == ProductionSafety::Disarmed &&
                status.stop_reason == ProductionStopReason::Stop &&
                !status.control_failsafe_stop,
            "real STOP CommandPayload must stop without heartbeat failsafe");
    require(status.last_ack_command_type ==
                static_cast<std::uint8_t>(boat::Type::Stop) &&
                status.last_ack_disposition == 0,
            "STOP must produce real CommandAckPayload");

    input.send_estop = true;
    system.step(700, input);
    input.send_estop = false;
    runRange(system, input, 710, 760);
    status = system.step(760, input);
    require(status.estop_received == 1 &&
                status.control_safety == ProductionSafety::EStop &&
                status.stop_reason == ProductionStopReason::EStop,
            "real ESTOP must enter E-STOP");

    input.send_clear_estop = true;
    system.step(900, input);
    input.send_clear_estop = false;
    runRange(system, input, 910, 960);
    status = system.step(960, input);
    require(status.clear_estop_received == 1 &&
                status.control_safety == ProductionSafety::Disarmed &&
                status.stop_reason == ProductionStopReason::None,
            "real CLEAR_ESTOP must return to DISARMED");
  }

  std::cout << "All phase-8 production boat protocol tests passed.\n";
  return 0;
}
