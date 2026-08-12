#include "ProductionProtocolSystem.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace cores3sim;
namespace fs = std::filesystem;

struct ScenarioRow {
  std::uint32_t t_ms{};
  ProductionProtocolInput input{};
};

static std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string value;
  while (std::getline(ss, value, ',')) out.push_back(value);
  return out;
}

static InjectedUartFault faultFrom(int value) {
  switch (value) {
    case 1: return InjectedUartFault::Drop;
    case 2: return InjectedUartFault::Corrupt;
    case 3: return InjectedUartFault::Framing;
    default: return InjectedUartFault::None;
  }
}

static std::vector<ScenarioRow> loadScenario(const fs::path& path) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("cannot open scenario: " + path.string());

  std::vector<ScenarioRow> rows;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#' || line.rfind("t_ms,", 0) == 0) {
      continue;
    }
    const auto c = split(line);
    if (c.size() != 14) {
      throw std::runtime_error(
          "phase-8 production protocol row needs 14 columns: " + line);
    }
    ScenarioRow r;
    r.t_ms = static_cast<std::uint32_t>(std::stoul(c[0]));
    r.input.comm_to_control_connected = std::stoi(c[1]) != 0;
    r.input.comm_to_control_latency_ms =
        static_cast<std::uint32_t>(std::stoul(c[2]));
    r.input.control_to_comm_connected = std::stoi(c[3]) != 0;
    r.input.control_to_comm_latency_ms =
        static_cast<std::uint32_t>(std::stoul(c[4]));
    r.input.comm_to_control_fault = faultFrom(std::stoi(c[5]));
    r.input.control_to_comm_fault = faultFrom(std::stoi(c[6]));
    r.input.gnss_valid = std::stoi(c[7]) != 0;
    r.input.send_arm = std::stoi(c[8]) != 0;
    r.input.send_start = std::stoi(c[9]) != 0;
    r.input.send_stop = std::stoi(c[10]) != 0;
    r.input.send_estop = std::stoi(c[11]) != 0;
    r.input.send_clear_estop = std::stoi(c[12]) != 0;
    r.input.send_disarm = std::stoi(c[13]) != 0;
    rows.push_back(r);
  }
  return rows;
}

static void clearOneShots(ProductionProtocolInput& input) {
  input.comm_to_control_fault = InjectedUartFault::None;
  input.control_to_comm_fault = InjectedUartFault::None;
  input.send_arm = false;
  input.send_start = false;
  input.send_stop = false;
  input.send_estop = false;
  input.send_clear_estop = false;
  input.send_disarm = false;
}

static std::string ageText(std::uint32_t value) {
  if (value == std::numeric_limits<std::uint32_t>::max()) return "never";
  return std::to_string(value) + "ms";
}

static const char* health(bool ok) { return ok ? "OK" : "BAD"; }

static const char* safetyName(ProductionSafety state) {
  switch (state) {
    case ProductionSafety::Boot: return "BOOT";
    case ProductionSafety::Disarmed: return "DISARMED";
    case ProductionSafety::Armed: return "ARMED";
    case ProductionSafety::Running: return "RUNNING";
    case ProductionSafety::EStop: return "E-STOP";
    case ProductionSafety::Fault: return "FAULT";
  }
  return "UNKNOWN";
}

static const char* reasonName(ProductionStopReason reason) {
  switch (reason) {
    case ProductionStopReason::None: return "NONE";
    case ProductionStopReason::Stop: return "STOP";
    case ProductionStopReason::EStop: return "E_STOP";
    case ProductionStopReason::Heartbeat: return "HEARTBEAT";
  }
  return "UNKNOWN";
}

static const char* typeName(std::uint8_t raw) {
  switch (static_cast<boat::Type>(raw)) {
    case boat::Type::Heartbeat: return "Heartbeat(32)";
    case boat::Type::Arm: return "Arm(33)";
    case boat::Type::Disarm: return "Disarm(34)";
    case boat::Type::StartTest: return "StartTest(35)";
    case boat::Type::Stop: return "Stop(36)";
    case boat::Type::Estop: return "Estop(37)";
    case boat::Type::ClearEstop: return "ClearEstop(38)";
    case boat::Type::CommandAck: return "CommandAck(17)";
    case boat::Type::GnssNavV2: return "GnssNavV2(59)";
    case boat::Type::ControlOutput: return "ControlOutput(62)";
    default: return raw ? "other" : "none";
  }
}

static void writeSvg(const fs::path& path,
                     const ProductionProtocolStatus& status,
                     const ProductionUartStats& uart,
                     const ProtocolDecoderStats& dec,
                     std::uint32_t t_ms) {
  std::ofstream out(path);
  out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"760\" height=\"360\">";
  out << "<rect width=\"760\" height=\"360\" fill=\"#101418\"/>";
  out << "<g fill=\"white\" font-family=\"monospace\">";
  out << "<text x=\"20\" y=\"30\" font-size=\"19\">Phase 8 - production boat_protocol / COBS + CRC32 / 921600 8N1</text>";
  out << "<rect x=\"20\" y=\"50\" width=\"345\" height=\"250\" rx=\"10\" fill=\"#263238\"/>";
  out << "<rect x=\"395\" y=\"50\" width=\"345\" height=\"250\" rx=\"10\" fill=\"#263238\"/>";

  out << "<text x=\"40\" y=\"80\" font-size=\"18\">COMM XIAO</text>";
  out << "<text x=\"40\" y=\"108\" font-size=\"13\">Control link: "
      << health(status.comm_link_fresh) << " age="
      << ageText(status.comm_link_age_ms) << "</text>";
  out << "<text x=\"40\" y=\"132\" font-size=\"13\">Control frames RX: "
      << status.control_frames_received << "</text>";
  out << "<text x=\"40\" y=\"156\" font-size=\"13\">Command ACK RX: "
      << status.command_acks_received << "</text>";
  out << "<text x=\"40\" y=\"180\" font-size=\"12\">Last RX: "
      << typeName(status.last_comm_rx_type) << " seq="
      << status.last_comm_rx_sequence << "</text>";
  out << "<text x=\"40\" y=\"204\" font-size=\"12\">Decoder CRC/COBS/LEN: "
      << dec.comm_crc_errors << '/' << dec.comm_cobs_errors << '/'
      << dec.comm_length_errors << "</text>";
  out << "<text x=\"40\" y=\"228\" font-size=\"12\">C->M wire drop/corrupt/frame: "
      << uart.control_to_comm.frames_dropped << '/'
      << uart.control_to_comm.corruptions_injected << '/'
      << uart.control_to_comm.framing_errors << "</text>";
  out << "<text x=\"40\" y=\"252\" font-size=\"12\">Last ACK type/disposition: "
      << static_cast<unsigned>(status.last_ack_command_type) << '/'
      << static_cast<unsigned>(status.last_ack_disposition) << "</text>";

  out << "<text x=\"415\" y=\"80\" font-size=\"18\">CONTROL XIAO</text>";
  out << "<text x=\"415\" y=\"108\" font-size=\"13\">Safety: "
      << safetyName(status.control_safety) << " reason="
      << reasonName(status.stop_reason) << "</text>";
  out << "<text x=\"415\" y=\"132\" font-size=\"13\">Host HB: "
      << health(status.control_host_heartbeat_fresh) << " age="
      << ageText(status.host_heartbeat_age_ms) << "</text>";
  out << "<text x=\"415\" y=\"156\" font-size=\"13\">RUN: "
      << (status.control_running ? "YES" : "NO") << " FAILSAFE: "
      << (status.control_failsafe_stop ? "YES" : "NO") << "</text>";
  out << "<text x=\"415\" y=\"180\" font-size=\"13\">GNSS_NAV_V2 RX: "
      << status.gnss_received << " payloadCRCerr="
      << status.gnss_canonical_crc_errors << "</text>";
  out << "<text x=\"415\" y=\"204\" font-size=\"12\">Last RX: "
      << typeName(status.last_control_rx_type) << " seq="
      << status.last_control_rx_sequence << "</text>";
  out << "<text x=\"415\" y=\"228\" font-size=\"12\">Decoder CRC/COBS/LEN: "
      << dec.control_crc_errors << '/' << dec.control_cobs_errors << '/'
      << dec.control_length_errors << "</text>";
  out << "<text x=\"415\" y=\"252\" font-size=\"12\">M->C wire drop/corrupt/frame: "
      << uart.comm_to_control.frames_dropped << '/'
      << uart.comm_to_control.corruptions_injected << '/'
      << uart.comm_to_control.framing_errors << "</text>";
  out << "<text x=\"415\" y=\"276\" font-size=\"12\">STOP/E/CLEAR RX: "
      << status.stop_received << '/' << status.estop_received << '/'
      << status.clear_estop_received << "</text>";

  out << "<text x=\"20\" y=\"330\" font-size=\"12\">t=" << t_ms
      << "ms Header=22B HB=12B Command=8B GNSSv2=68B delimiter=0x00</text>";
  out << "</g></svg>";
}

int main(int argc, char** argv) {
  const fs::path scenario = argc > 1
      ? argv[1]
      : "scenarios/production_protocol_faults.csv";
  const fs::path outdir = argc > 2
      ? argv[2]
      : "artifacts/production_protocol";
  fs::create_directories(outdir);

  const auto rows = loadScenario(scenario);
  if (rows.empty()) throw std::runtime_error("scenario is empty");

  ProductionProtocolSystem system;
  ProductionProtocolInput current = rows.front().input;
  std::uint32_t last_t = rows.front().t_ms;
  ProductionProtocolStatus status = system.step(last_t, current);
  clearOneShots(current);

  std::ofstream trace(outdir / "trace.csv");
  trace << "t_ms,safety,stop_reason,running,failsafe,comm_link_fresh,"
           "comm_link_age_ms,host_hb_fresh,host_hb_age_ms,gnss_rx,"
           "control_frames_rx,acks_rx,stop_rx,estop_rx,clear_estop_rx,"
           "arm_rx,start_rx,disarm_rx,gnss_payload_crc_errors,"
           "last_comm_rx_type,last_control_rx_type,last_ack_type,last_ack_disp,"
           "comm_dec_crc,comm_dec_cobs,comm_dec_len,"
           "control_dec_crc,control_dec_cobs,control_dec_len,"
           "c2c_sent,c2c_delivered,c2c_dropped,c2c_corrupt,c2c_framing,"
           "c2m_sent,c2m_delivered,c2m_dropped,c2m_corrupt,c2m_framing\n";

  for (std::size_t index = 0; index < rows.size(); ++index) {
    if (index > 0) {
      for (std::uint32_t t = last_t + 10; t < rows[index].t_ms; t += 10) {
        system.step(t, current);
      }
      current = rows[index].input;
      status = system.step(rows[index].t_ms, current);
      last_t = rows[index].t_ms;
      clearOneShots(current);
    }

    const auto& uart = system.uartStats();
    const auto dec = system.decoderStats();
    std::ostringstream filename;
    filename << "frame_" << std::setw(3) << std::setfill('0') << index
             << '_' << rows[index].t_ms << "ms.svg";
    writeSvg(outdir / filename.str(), status, uart, dec, rows[index].t_ms);

    trace << rows[index].t_ms << ','
          << static_cast<unsigned>(status.control_safety) << ','
          << static_cast<unsigned>(status.stop_reason) << ','
          << status.control_running << ',' << status.control_failsafe_stop << ','
          << status.comm_link_fresh << ',' << status.comm_link_age_ms << ','
          << status.control_host_heartbeat_fresh << ','
          << status.host_heartbeat_age_ms << ',' << status.gnss_received << ','
          << status.control_frames_received << ','
          << status.command_acks_received << ',' << status.stop_received << ','
          << status.estop_received << ',' << status.clear_estop_received << ','
          << status.arm_received << ',' << status.start_received << ','
          << status.disarm_received << ','
          << status.gnss_canonical_crc_errors << ','
          << static_cast<unsigned>(status.last_comm_rx_type) << ','
          << static_cast<unsigned>(status.last_control_rx_type) << ','
          << static_cast<unsigned>(status.last_ack_command_type) << ','
          << static_cast<unsigned>(status.last_ack_disposition) << ','
          << dec.comm_crc_errors << ',' << dec.comm_cobs_errors << ','
          << dec.comm_length_errors << ',' << dec.control_crc_errors << ','
          << dec.control_cobs_errors << ',' << dec.control_length_errors << ','
          << uart.comm_to_control.frames_sent << ','
          << uart.comm_to_control.frames_delivered << ','
          << uart.comm_to_control.frames_dropped << ','
          << uart.comm_to_control.corruptions_injected << ','
          << uart.comm_to_control.framing_errors << ','
          << uart.control_to_comm.frames_sent << ','
          << uart.control_to_comm.frames_delivered << ','
          << uart.control_to_comm.frames_dropped << ','
          << uart.control_to_comm.corruptions_injected << ','
          << uart.control_to_comm.framing_errors << '\n';

    std::cout << rows[index].t_ms << "ms safety="
              << safetyName(status.control_safety)
              << " comm=" << health(status.comm_link_fresh)
              << " hostHB=" << health(status.control_host_heartbeat_fresh)
              << " run=" << status.control_running
              << " failsafe=" << status.control_failsafe_stop << '\n';
  }
  return 0;
}
