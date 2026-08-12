#include "ProductionCommandTelemetrySystem.h"

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
  ProductionCommandTelemetryInput input{};
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

static ProductionAction actionFrom(int value) {
  if (value < 0 || value > 10) return ProductionAction::None;
  return static_cast<ProductionAction>(value);
}

static std::vector<ScenarioRow> loadScenario(const fs::path& path) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("cannot open scenario: " + path.string());
  std::vector<ScenarioRow> rows;
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#' || line.rfind("t_ms,", 0) == 0) continue;
    const auto c = split(line);
    if (c.size() != 19) {
      throw std::runtime_error("phase-9 row needs 19 columns: " + line);
    }
    ScenarioRow r;
    r.t_ms = static_cast<std::uint32_t>(std::stoul(c[0]));
    r.input.comm_to_control_connected = std::stoi(c[1]) != 0;
    r.input.comm_to_control_latency_ms = static_cast<std::uint32_t>(std::stoul(c[2]));
    r.input.control_to_comm_connected = std::stoi(c[3]) != 0;
    r.input.control_to_comm_latency_ms = static_cast<std::uint32_t>(std::stoul(c[4]));
    r.input.comm_to_control_fault = faultFrom(std::stoi(c[5]));
    r.input.control_to_comm_fault = faultFrom(std::stoi(c[6]));
    r.input.gnss_valid = std::stoi(c[7]) != 0;
    r.input.action = actionFrom(std::stoi(c[8]));
    r.input.mode = static_cast<std::uint8_t>(std::stoul(c[9]));
    r.input.manual_mask = static_cast<std::uint8_t>(std::stoul(c[10]));
    r.input.manual_left = std::stof(c[11]);
    r.input.manual_right = std::stof(c[12]);
    r.input.manual_rear = std::stof(c[13]);
    r.input.manual_propulsion = std::stof(c[14]);
    r.input.heading_rad = std::stof(c[15]);
    r.input.waypoint_latitude_deg = std::stod(c[16]);
    r.input.waypoint_longitude_deg = std::stod(c[17]);
    r.input.corrupt_canonical_crc = std::stoi(c[18]) != 0;
    rows.push_back(r);
  }
  return rows;
}

static void clearOneShots(ProductionCommandTelemetryInput& input) {
  input.action = ProductionAction::None;
  input.comm_to_control_fault = InjectedUartFault::None;
  input.control_to_comm_fault = InjectedUartFault::None;
  input.corrupt_canonical_crc = false;
}

static const char* displaySafetyName(production_control::AuthoritativeSafety s) {
  return production_control::safetyName(s);
}

static const char* displayModeName(production_control::ControlMode m) {
  return production_control::modeName(m);
}

static const char* pendingName(PendingCommandState s) {
  switch (s) {
    case PendingCommandState::Idle: return "IDLE";
    case PendingCommandState::Waiting: return "WAIT";
    case PendingCommandState::Accepted: return "ACCEPT";
    case PendingCommandState::Rejected: return "REJECT";
    case PendingCommandState::Timeout: return "TIMEOUT";
  }
  return "?";
}

static std::string ageText(std::uint32_t age) {
  if (age == std::numeric_limits<std::uint32_t>::max()) return "never";
  return std::to_string(age) + "ms";
}

static void writeSvg(const fs::path& path,
                     const ProductionCommandTelemetryStatus& s,
                     const ProductionCommandTelemetryStats& stats,
                     const ProductionUartStats& uart,
                     const Phase9DecoderStats& dec,
                     std::uint32_t t_ms) {
  std::ofstream out(path);
  out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"900\" height=\"450\">";
  out << "<rect width=\"900\" height=\"450\" fill=\"#101418\"/>";
  out << "<g fill=\"white\" font-family=\"monospace\">";
  out << "<text x=\"20\" y=\"30\" font-size=\"19\">Phase 9 - production CommandIngress / retry / telemetry</text>";
  out << "<rect x=\"20\" y=\"50\" width=\"420\" height=\"345\" rx=\"10\" fill=\"#263238\"/>";
  out << "<rect x=\"460\" y=\"50\" width=\"420\" height=\"345\" rx=\"10\" fill=\"#263238\"/>";

  out << "<text x=\"40\" y=\"80\" font-size=\"18\">COMM XIAO</text>";
  out << "<text x=\"40\" y=\"106\" font-size=\"13\">link=" << (s.comm_link_fresh?"OK":"BAD")
      << " age=" << ageText(s.comm_link_age_ms) << "</text>";
  out << "<text x=\"40\" y=\"130\" font-size=\"13\">pending=" << pendingName(s.pending_state)
      << " type=" << static_cast<unsigned>(s.pending_type) << " attempts=" << static_cast<unsigned>(s.pending_attempts) << "</text>";
  out << "<text x=\"40\" y=\"154\" font-size=\"12\">request/seq=" << s.pending_request_id << '/' << s.pending_sequence << "</text>";
  out << "<text x=\"40\" y=\"178\" font-size=\"12\">ACK disp/reason=" << static_cast<unsigned>(s.last_control_ack_disposition)
      << '/' << s.last_control_ack_reason << " duplicateRX=" << s.duplicate_acks_rx << "</text>";
  out << "<text x=\"40\" y=\"202\" font-size=\"12\">retry/accept/reject/timeout=" << stats.pending_retries << '/'
      << stats.pending_accepted << '/' << stats.pending_rejected << '/' << stats.pending_timeouts << "</text>";
  out << "<text x=\"40\" y=\"232\" font-size=\"13\">Telemetry RX</text>";
  out << "<text x=\"40\" y=\"254\" font-size=\"11\">OUT/SNAP/INA/VESC/ACT/HEALTH="
      << s.control_output_rx << '/' << s.control_snapshot_rx << '/' << s.ina_status_rx << '/'
      << s.vesc_telemetry_rx << '/' << s.actuator_state_rx << '/' << s.system_health_rx << "</text>";
  out << "<text x=\"40\" y=\"278\" font-size=\"11\">INA=" << std::fixed << std::setprecision(2)
      << s.latest_ina_bus_v << "V " << s.latest_ina_current_a << "A  VESC=" << s.latest_vesc_erpm << "erpm</text>";
  out << "<text x=\"40\" y=\"302\" font-size=\"11\">PCA=" << static_cast<unsigned>(s.latest_actuator_pca_ready)
      << " outputs=" << static_cast<unsigned>(s.latest_actuator_outputs_enabled)
      << " health=0x" << std::hex << s.latest_health_flags << std::dec << "</text>";
  out << "<text x=\"40\" y=\"330\" font-size=\"11\">COMM decoder CRC/COBS/LEN=" << dec.comm_crc_errors << '/'
      << dec.comm_cobs_errors << '/' << dec.comm_length_errors << "</text>";
  out << "<text x=\"40\" y=\"354\" font-size=\"11\">C2M drop/corrupt/frame=" << uart.control_to_comm.frames_dropped << '/'
      << uart.control_to_comm.corruptions_injected << '/' << uart.control_to_comm.framing_errors << "</text>";

  out << "<text x=\"480\" y=\"80\" font-size=\"18\">CONTROL XIAO</text>";
  out << "<text x=\"480\" y=\"106\" font-size=\"13\">safety=" << displaySafetyName(s.safety)
      << " mode=" << displayModeName(s.control_mode) << " run=" << (s.running?"YES":"NO") << "</text>";
  out << "<text x=\"480\" y=\"130\" font-size=\"13\">hostHB=" << (s.host_heartbeat_fresh?"OK":"BAD")
      << " age=" << ageText(s.host_heartbeat_age_ms) << " failsafe=" << (s.failsafe_stop?"YES":"NO") << "</text>";
  out << "<text x=\"480\" y=\"158\" font-size=\"13\">CommandIngress</text>";
  out << "<text x=\"480\" y=\"182\" font-size=\"11\">new/applied/rejected/duplicate=" << s.ingress_new << '/'
      << s.ingress_applied << '/' << s.ingress_rejected << '/' << s.ingress_duplicates << "</text>";
  out << "<text x=\"480\" y=\"206\" font-size=\"11\">conflict/stale/malformed=" << s.ingress_conflicts << '/'
      << s.ingress_stale << '/' << s.ingress_malformed << "</text>";
  out << "<text x=\"480\" y=\"234\" font-size=\"13\">Waypoint revision/count=" << s.waypoint_revision
      << '/' << static_cast<unsigned>(s.waypoint_count) << "</text>";
  out << "<text x=\"480\" y=\"258\" font-size=\"12\">GNSS RX=" << s.gnss_nav_rx
      << " payloadCRCerr=" << s.gnss_payload_crc_errors << "</text>";
  out << "<text x=\"480\" y=\"282\" font-size=\"12\">manual mask=" << static_cast<unsigned>(s.manual_output_mask)
      << " refresh=" << stats.manual_refreshes_sent << " telemetry=" << stats.telemetry_bursts_sent << "</text>";
  out << "<text x=\"480\" y=\"330\" font-size=\"11\">CONTROL decoder CRC/COBS/LEN=" << dec.control_crc_errors << '/'
      << dec.control_cobs_errors << '/' << dec.control_length_errors << "</text>";
  out << "<text x=\"480\" y=\"354\" font-size=\"11\">M2C drop/corrupt/frame=" << uart.comm_to_control.frames_dropped << '/'
      << uart.comm_to_control.corruptions_injected << '/' << uart.comm_to_control.framing_errors << "</text>";

  out << "<text x=\"20\" y=\"430\" font-size=\"12\">t=" << t_ms
      << "ms retry=100ms timeout=1200ms replayWindow=64 telemetry=100ms manualRefresh=200ms</text>";
  out << "</g></svg>";
}

int main(int argc, char** argv) {
  const fs::path scenario = argc > 1 ? argv[1] : "scenarios/production_command_retry.csv";
  const fs::path outdir = argc > 2 ? argv[2] : "artifacts/production_command_retry";
  fs::create_directories(outdir);
  const auto rows = loadScenario(scenario);
  if (rows.empty()) throw std::runtime_error("scenario is empty");

  ProductionCommandTelemetrySystem system;
  ProductionCommandTelemetryInput current = rows.front().input;
  std::uint32_t last_t = rows.front().t_ms;
  auto status = system.stepWithManualRefresh(last_t, current);
  clearOneShots(current);

  std::ofstream trace(outdir / "trace.csv");
  trace << "t_ms,safety,mode,running,failsafe,comm_fresh,comm_age_ms,hb_fresh,hb_age_ms,"
           "pending_state,pending_type,pending_request,pending_seq,pending_attempts,last_ack_disp,last_ack_reason,"
           "ingress_new,ingress_applied,ingress_rejected,ingress_duplicate,ingress_conflict,ingress_stale,ingress_malformed,"
           "wp_revision,wp_count,out_rx,snapshot_rx,ina_rx,vesc_rx,act_rx,health_rx,gnss_rx,duplicate_ack_rx,"
           "retry_count,accepted_count,rejected_count,timeout_count,manual_refreshes,telemetry_bursts,"
           "comm_dec_crc,comm_dec_cobs,comm_dec_len,control_dec_crc,control_dec_cobs,control_dec_len\n";

  for (std::size_t i = 0; i < rows.size(); ++i) {
    if (i > 0) {
      for (std::uint32_t t = last_t + 10; t < rows[i].t_ms; t += 10) {
        system.stepWithManualRefresh(t, current);
      }
      current = rows[i].input;
      status = system.stepWithManualRefresh(rows[i].t_ms, current);
      last_t = rows[i].t_ms;
      clearOneShots(current);
    }
    const auto& stats = system.stats();
    const auto& uart = system.uartStats();
    const auto dec = system.decoderStats();
    std::ostringstream name;
    name << "frame_" << std::setw(3) << std::setfill('0') << i << '_' << rows[i].t_ms << "ms.svg";
    writeSvg(outdir / name.str(), status, stats, uart, dec, rows[i].t_ms);
    trace << rows[i].t_ms << ',' << static_cast<unsigned>(status.safety) << ','
          << static_cast<unsigned>(status.control_mode) << ',' << status.running << ',' << status.failsafe_stop << ','
          << status.comm_link_fresh << ',' << status.comm_link_age_ms << ',' << status.host_heartbeat_fresh << ','
          << status.host_heartbeat_age_ms << ',' << static_cast<unsigned>(status.pending_state) << ','
          << static_cast<unsigned>(status.pending_type) << ',' << status.pending_request_id << ',' << status.pending_sequence << ','
          << static_cast<unsigned>(status.pending_attempts) << ',' << static_cast<unsigned>(status.last_control_ack_disposition) << ','
          << status.last_control_ack_reason << ',' << status.ingress_new << ',' << status.ingress_applied << ','
          << status.ingress_rejected << ',' << status.ingress_duplicates << ',' << status.ingress_conflicts << ','
          << status.ingress_stale << ',' << status.ingress_malformed << ',' << status.waypoint_revision << ','
          << static_cast<unsigned>(status.waypoint_count) << ',' << status.control_output_rx << ',' << status.control_snapshot_rx << ','
          << status.ina_status_rx << ',' << status.vesc_telemetry_rx << ',' << status.actuator_state_rx << ','
          << status.system_health_rx << ',' << status.gnss_nav_rx << ',' << status.duplicate_acks_rx << ','
          << stats.pending_retries << ',' << stats.pending_accepted << ',' << stats.pending_rejected << ','
          << stats.pending_timeouts << ',' << stats.manual_refreshes_sent << ',' << stats.telemetry_bursts_sent << ','
          << dec.comm_crc_errors << ',' << dec.comm_cobs_errors << ',' << dec.comm_length_errors << ','
          << dec.control_crc_errors << ',' << dec.control_cobs_errors << ',' << dec.control_length_errors << '\n';
    std::cout << rows[i].t_ms << "ms safety=" << displaySafetyName(status.safety)
              << " pending=" << pendingName(status.pending_state)
              << " attempts=" << static_cast<unsigned>(status.pending_attempts)
              << " dup=" << status.ingress_duplicates
              << " refresh=" << stats.manual_refreshes_sent
              << " telemetry=" << status.control_snapshot_rx << '\n';
  }
  return 0;
}
