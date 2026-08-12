#include "DualControllerSystem.h"

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
  DualControllerInput input{};
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
  if (!file) {
    throw std::runtime_error("cannot open scenario: " + path.string());
  }

  std::vector<ScenarioRow> rows;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#' || line.rfind("t_ms,", 0) == 0) {
      continue;
    }

    const auto columns = split(line);
    if (columns.size() != 9) {
      throw std::runtime_error(
          "phase-7 scenario row needs 9 columns: " + line);
    }

    ScenarioRow row;
    row.t_ms = static_cast<std::uint32_t>(std::stoul(columns[0]));
    row.input.comm_to_control_connected = std::stoi(columns[1]) != 0;
    row.input.control_to_comm_connected = std::stoi(columns[2]) != 0;
    row.input.comm_to_control_latency_ms =
        static_cast<std::uint32_t>(std::stoul(columns[3]));
    row.input.control_to_comm_latency_ms =
        static_cast<std::uint32_t>(std::stoul(columns[4]));
    row.input.comm_to_control_fault = faultFrom(std::stoi(columns[5]));
    row.input.control_to_comm_fault = faultFrom(std::stoi(columns[6]));
    row.input.gnss_valid = std::stoi(columns[7]) != 0;
    row.input.send_stop = std::stoi(columns[8]) != 0;
    rows.push_back(row);
  }
  return rows;
}

static std::string ageText(std::uint32_t value) {
  if (value == std::numeric_limits<std::uint32_t>::max()) return "never";
  return std::to_string(value) + "ms";
}

static const char* health(bool ok) { return ok ? "OK" : "BAD"; }

static void writeSvg(const fs::path& path,
                     const DualControllerStatus& status,
                     const DuplexUartStats& uart,
                     std::uint32_t t_ms) {
  std::ofstream out(path);
  out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"640\" height=\"300\">";
  out << "<rect width=\"640\" height=\"300\" fill=\"#101418\"/>";
  out << "<g fill=\"white\" font-family=\"monospace\">";
  out << "<text x=\"20\" y=\"30\" font-size=\"20\">Phase 7 - Dual Controller UART 921600 8N1</text>";
  out << "<rect x=\"20\" y=\"50\" width=\"280\" height=\"210\" rx=\"10\" fill=\"#263238\"/>";
  out << "<rect x=\"340\" y=\"50\" width=\"280\" height=\"210\" rx=\"10\" fill=\"#263238\"/>";

  out << "<text x=\"40\" y=\"80\" font-size=\"18\">COMM controller</text>";
  out << "<text x=\"40\" y=\"110\" font-size=\"14\">Control link: "
      << health(status.comm_sees_control_link) << "</text>";
  out << "<text x=\"40\" y=\"135\" font-size=\"14\">HB age: "
      << ageText(status.control_heartbeat_age_ms) << "</text>";
  out << "<text x=\"40\" y=\"160\" font-size=\"14\">Results RX: "
      << status.results_received << "</text>";
  out << "<text x=\"40\" y=\"185\" font-size=\"14\">C2M dropped: "
      << uart.control_to_comm.packets_dropped << "</text>";

  out << "<text x=\"360\" y=\"80\" font-size=\"18\">CONTROL controller</text>";
  out << "<text x=\"360\" y=\"110\" font-size=\"14\">Comm link: "
      << health(status.control_sees_comm_link) << "</text>";
  out << "<text x=\"360\" y=\"135\" font-size=\"14\">HB age: "
      << ageText(status.comm_heartbeat_age_ms) << "</text>";
  out << "<text x=\"360\" y=\"160\" font-size=\"14\">GNSS RX: "
      << status.gnss_received << "</text>";
  out << "<text x=\"360\" y=\"185\" font-size=\"14\">STOP RX: "
      << status.stop_received << "</text>";
  out << "<text x=\"360\" y=\"210\" font-size=\"14\">RUN: "
      << (status.control_running ? "YES" : "NO") << "</text>";
  out << "<text x=\"360\" y=\"235\" font-size=\"14\">FAILSAFE: "
      << (status.control_failsafe_stop ? "YES" : "NO") << "</text>";

  out << "<text x=\"20\" y=\"285\" font-size=\"13\">t=" << t_ms
      << "ms  C2C drop=" << uart.comm_to_control.packets_dropped
      << " crc=" << uart.comm_to_control.crc_errors
      << " frame=" << uart.comm_to_control.framing_errors << "</text>";
  out << "</g></svg>";
}

int main(int argc, char** argv) {
  const fs::path scenario =
      argc > 1 ? argv[1] : "scenarios/dual_uart_faults.csv";
  const fs::path outdir =
      argc > 2 ? argv[2] : "artifacts/dual_uart";
  fs::create_directories(outdir);

  const auto rows = loadScenario(scenario);
  if (rows.empty()) throw std::runtime_error("scenario is empty");

  DualControllerSystem system;
  DualControllerInput current = rows.front().input;
  std::uint32_t last_t = rows.front().t_ms;

  std::ofstream trace(outdir / "trace.csv");
  trace << "t_ms,control_running,failsafe,comm_link_ok,control_link_ok,"
           "comm_hb_age_ms,control_hb_age_ms,gnss_rx,results_rx,stop_rx,"
           "c2c_sent,c2c_delivered,c2c_dropped,c2c_crc,c2c_framing,"
           "c2m_sent,c2m_delivered,c2m_dropped,c2m_crc,c2m_framing\n";

  DualControllerStatus status = system.step(last_t, current);
  current.comm_to_control_fault = InjectedUartFault::None;
  current.control_to_comm_fault = InjectedUartFault::None;
  current.send_stop = false;

  for (std::size_t index = 0; index < rows.size(); ++index) {
    if (index > 0) {
      // Advance the deterministic controllers at 10 ms resolution between
      // visible scenario events, but only emit an SVG at each event row.
      for (std::uint32_t t = last_t + 10; t < rows[index].t_ms; t += 10) {
        system.step(t, current);
      }

      current = rows[index].input;
      status = system.step(rows[index].t_ms, current);
      last_t = rows[index].t_ms;
      current.comm_to_control_fault = InjectedUartFault::None;
      current.control_to_comm_fault = InjectedUartFault::None;
      current.send_stop = false;
    }

    const auto& uart = system.uartStats();
    std::ostringstream filename;
    filename << "frame_" << std::setw(3) << std::setfill('0') << index
             << '_' << rows[index].t_ms << "ms.svg";
    writeSvg(outdir / filename.str(), status, uart, rows[index].t_ms);

    trace << rows[index].t_ms << ','
          << status.control_running << ','
          << status.control_failsafe_stop << ','
          << status.comm_sees_control_link << ','
          << status.control_sees_comm_link << ','
          << status.comm_heartbeat_age_ms << ','
          << status.control_heartbeat_age_ms << ','
          << status.gnss_received << ','
          << status.results_received << ','
          << status.stop_received << ','
          << uart.comm_to_control.packets_sent << ','
          << uart.comm_to_control.packets_delivered << ','
          << uart.comm_to_control.packets_dropped << ','
          << uart.comm_to_control.crc_errors << ','
          << uart.comm_to_control.framing_errors << ','
          << uart.control_to_comm.packets_sent << ','
          << uart.control_to_comm.packets_delivered << ','
          << uart.control_to_comm.packets_dropped << ','
          << uart.control_to_comm.crc_errors << ','
          << uart.control_to_comm.framing_errors << '\n';

    std::cout << rows[index].t_ms
              << "ms COMM=" << health(status.comm_sees_control_link)
              << " CONTROL=" << health(status.control_sees_comm_link)
              << " RUN=" << status.control_running
              << " FAILSAFE=" << status.control_failsafe_stop << '\n';
  }

  return 0;
}
