#include "App.h"
#include "VirtualHardware.h"

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
  VirtualHwInput hw{};
  TouchSample touch{};
};

static std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string v;
  while (std::getline(ss, v, ',')) out.push_back(v);
  return out;
}

static void parsePhase3Base(const std::vector<std::string>& c,
                            ScenarioRow& r) {
  r.t_ms = static_cast<std::uint32_t>(std::stoul(c[0]));
  r.hw.pitch_deg = std::stof(c[1]);
  r.hw.battery_v = std::stof(c[2]);
  r.hw.imu_online = std::stoi(c[3]) != 0;
  r.hw.i2c_connected = std::stoi(c[4]) != 0;
  r.hw.i2c_latency_ms = static_cast<std::uint32_t>(std::stoul(c[5]));
  r.hw.i2c_nack = std::stoi(c[6]) != 0;
  r.hw.uart_connected = std::stoi(c[7]) != 0;
  r.hw.uart_latency_ms = static_cast<std::uint32_t>(std::stoul(c[8]));
  r.hw.gnss_source_valid = std::stoi(c[9]) != 0;
  r.hw.uart_corrupt_byte = std::stoi(c[10]) != 0;
  r.hw.uart_crc_error = std::stoi(c[11]) != 0;
  r.hw.uart_framing_error = std::stoi(c[12]) != 0;
  r.hw.loop_jitter_ms = static_cast<std::int32_t>(std::stol(c[13]));
  r.hw.sd_connected = std::stoi(c[14]) != 0;
  r.hw.sd_fail_write = std::stoi(c[15]) != 0;
  r.hw.sd_latency_ms = static_cast<std::uint32_t>(std::stoul(c[16]));
  r.touch = {std::stoi(c[17]) != 0, std::stoi(c[18]), std::stoi(c[19])};
}

static std::vector<ScenarioRow> loadScenario(const fs::path& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot open scenario: " + path.string());

  std::vector<ScenarioRow> rows;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#' || line.rfind("t_ms,", 0) == 0) {
      continue;
    }

    const auto c = split(line);
    ScenarioRow r;

    if (c.size() == 7) {
      r.t_ms = static_cast<std::uint32_t>(std::stoul(c[0]));
      r.hw.pitch_deg = std::stof(c[1]);
      r.hw.battery_v = std::stof(c[2]);
      r.hw.imu_online = std::stoi(c[3]) != 0;
      r.touch = {std::stoi(c[4]) != 0, std::stoi(c[5]), std::stoi(c[6])};
    } else if (c.size() == 12) {
      r.t_ms = static_cast<std::uint32_t>(std::stoul(c[0]));
      r.hw.pitch_deg = std::stof(c[1]);
      r.hw.battery_v = std::stof(c[2]);
      r.hw.imu_online = std::stoi(c[3]) != 0;
      r.hw.i2c_connected = std::stoi(c[4]) != 0;
      r.hw.i2c_latency_ms = static_cast<std::uint32_t>(std::stoul(c[5]));
      r.hw.uart_connected = std::stoi(c[6]) != 0;
      r.hw.uart_latency_ms = static_cast<std::uint32_t>(std::stoul(c[7]));
      r.hw.gnss_source_valid = std::stoi(c[8]) != 0;
      r.touch = {std::stoi(c[9]) != 0, std::stoi(c[10]), std::stoi(c[11])};
    } else if (c.size() == 20) {
      parsePhase3Base(c, r);
    } else if (c.size() == 27) {
      parsePhase3Base(c, r);
      r.hw.bno_model_enabled = std::stoi(c[20]) != 0;
      r.hw.bno_force_reset = std::stoi(c[21]) != 0;
      r.hw.bno_reports_enabled = std::stoi(c[22]) != 0;
      r.hw.bno_stall_reports = std::stoi(c[23]) != 0;
      r.hw.bno_report_interval_ms =
          static_cast<std::uint32_t>(std::stoul(c[24]));
      r.hw.bno_reinit_delay_ms =
          static_cast<std::uint32_t>(std::stoul(c[25]));
      r.hw.bno_auto_reinit = std::stoi(c[26]) != 0;
    } else {
      throw std::runtime_error(
          "scenario row needs 7, 12, 20, or 27 columns: " + line);
    }

    rows.push_back(r);
  }

  return rows;
}

static std::string xml(std::string s) {
  std::string o;
  for (const char ch : s) {
    if (ch == '&') o += "&amp;";
    else if (ch == '<') o += "&lt;";
    else if (ch == '>') o += "&gt;";
    else o += ch;
  }
  return o;
}

static const char* health(bool ok) { return ok ? "OK" : "BAD"; }

static const char* bnoState(const UiFrame& f) {
  if (!f.bno_model_active) return "N/A";
  if (f.bno_reinitializing) return "INIT";
  if (!f.bno_initialized) return "OFF";
  if (!f.bno_report_fresh) return "STALE";
  return "RUN";
}

static void writeSvg(const fs::path& path, const UiFrame& f,
                     const SensorSample& s, std::uint32_t t_ms) {
  const char* state_color =
      f.state == RunState::Fault ? "#c62828" :
      f.state == RunState::Running ? "#1565c0" :
      f.state == RunState::Ready ? "#00796b" : "#666666";

  std::ofstream o(path);
  o << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"320\" height=\"240\">";
  o << "<rect width=\"320\" height=\"240\" fill=\"#101418\"/>";
  o << "<rect width=\"320\" height=\"36\" fill=\"" << state_color << "\"/>";
  o << "<g fill=\"white\" font-family=\"monospace\">";
  o << "<text x=\"9\" y=\"24\" font-size=\"17\">CoreS3 DEVICE SIM</text>";
  o << "<text x=\"309\" y=\"24\" text-anchor=\"end\" font-size=\"15\">"
    << App::stateName(f.state) << "</text>";
  o << "<text x=\"10\" y=\"56\" font-size=\"13\">"
    << xml(f.message) << "</text>";

  if (!f.warning.empty()) {
    o << "<text x=\"10\" y=\"74\" fill=\"#ffca28\" font-size=\"10\">WARN: "
      << xml(f.warning) << "</text>";
  }

  o << "<text x=\"10\" y=\"95\" font-size=\"11\">I2C:"
    << health(f.i2c_ok) << " BNO:" << bnoState(f)
    << " UART:" << health(f.uart_ok) << "</text>";
  o << "<text x=\"10\" y=\"114\" font-size=\"11\">GNSS:"
    << health(f.gnss_ok) << " FRAME:" << health(f.uart_frame_ok)
    << " SD:" << health(f.sd_ok) << "</text>";
  o << "<text x=\"10\" y=\"133\" font-size=\"11\">TIME:"
    << health(f.timing_ok) << " J:" << f.loop_jitter_ms << "ms Pitch:"
    << std::fixed << std::setprecision(1) << f.pitch_deg << "deg</text>";
  o << "<text x=\"10\" y=\"152\" font-size=\"10\">t:" << t_ms
    << " ticks:" << f.run_ticks << " BNO-age:";

  if (!f.bno_model_active) {
    o << "n/a";
  } else if (s.bno_report_age_ms ==
             std::numeric_limits<std::uint32_t>::max()) {
    o << "never";
  } else {
    o << s.bno_report_age_ms << "ms";
  }
  o << "</text>";

  o << "<rect x=\"60\" y=\"177\" width=\"200\" height=\"52\" rx=\"8\" fill=\"#37474f\"/>";
  o << "<text x=\"160\" y=\"209\" text-anchor=\"middle\" font-size=\"19\">"
    << xml(f.button_label) << "</text></g></svg>";
}

int main(int argc, char** argv) {
  const fs::path scenario = argc > 1 ? argv[1] : "scenarios/demo.csv";
  const fs::path outdir = argc > 2 ? argv[2] : "artifacts";
  fs::create_directories(outdir);

  const auto rows = loadScenario(scenario);
  if (rows.empty()) throw std::runtime_error("scenario is empty");

  App app;
  VirtualHardware hw;
  std::ofstream trace(outdir / "trace.csv");
  trace << "t_ms,state,pitch_deg,i2c_ok,i2c_nack,imu_ok,"
           "uart_ok,uart_frame_ok,gnss_ok,gnss_age_ms,timing_ok,jitter_ms,"
           "sd_ok,bno_active,bno_initialized,bno_reinitializing,"
           "bno_report_fresh,bno_report_age_ms,run_ticks,warning,"
           "i2c_timeouts,i2c_nacks,uart_dropped,uart_corrupt,uart_crc_errors,"
           "uart_framing_errors,jitter_events,max_jitter_ms,sd_failures,"
           "sd_timeouts,bno_resets,bno_reinit_attempts,bno_reinit_successes,"
           "bno_reports,bno_stale_events\n";

  for (std::size_t i = 0; i < rows.size(); ++i) {
    const auto& r = rows[i];
    hw.apply(r.t_ms, r.hw);
    const auto sensor = hw.sample(r.t_ms);
    const auto frame = app.update(r.t_ms, sensor, r.touch);

    std::ostringstream filename;
    filename << "frame_" << std::setw(3) << std::setfill('0') << i
             << '_' << r.t_ms << "ms.svg";
    writeSvg(outdir / filename.str(), frame, sensor, r.t_ms);

    const auto& st = hw.stats();
    const auto& bs = hw.bnoStats();
    trace << r.t_ms << ',' << App::stateName(frame.state) << ','
          << sensor.pitch_deg << ',' << sensor.i2c_ok << ','
          << sensor.i2c_nack << ',' << sensor.imu_ok << ','
          << sensor.uart_ok << ',' << sensor.uart_frame_ok << ','
          << sensor.gnss_ok << ',' << sensor.gnss_age_ms << ','
          << sensor.timing_ok << ',' << sensor.loop_jitter_ms << ','
          << sensor.sd_ok << ',' << sensor.bno_model_active << ','
          << sensor.bno_initialized << ',' << sensor.bno_reinitializing << ','
          << sensor.bno_report_fresh << ',' << sensor.bno_report_age_ms << ','
          << frame.run_ticks << ",\"" << frame.warning << "\","
          << st.i2c_timeouts << ',' << st.i2c_nacks << ','
          << st.uart_frames_dropped << ',' << st.uart_corrupt_bytes << ','
          << st.uart_crc_errors << ',' << st.uart_framing_errors << ','
          << st.timing_jitter_events << ',' << st.max_abs_jitter_ms << ','
          << st.sd_write_failures << ',' << st.sd_write_timeouts << ','
          << bs.resets << ',' << bs.reinit_attempts << ','
          << bs.reinit_successes << ',' << bs.reports_delivered << ','
          << bs.stale_events << '\n';

    std::cout << r.t_ms << "ms " << App::stateName(frame.state)
              << " I2C=" << health(sensor.i2c_ok)
              << " BNO=" << bnoState(frame)
              << " UART=" << health(sensor.uart_ok)
              << " SD=" << health(sensor.sd_ok) << '\n';
  }

  return 0;
}
