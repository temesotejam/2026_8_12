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

static std::vector<ScenarioRow> loadScenario(const fs::path& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot open scenario: " + path.string());
  std::vector<ScenarioRow> rows;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#' || line.rfind("t_ms,", 0) == 0) continue;
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
    } else {
      throw std::runtime_error("scenario row needs 7 or 12 columns: " + line);
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

static void writeSvg(const fs::path& path, const UiFrame& frame,
                     const SensorSample& sensor, std::uint32_t t_ms) {
  const char* state_color = frame.state == RunState::Fault ? "#c62828" :
                            frame.state == RunState::Running ? "#1565c0" :
                            frame.state == RunState::Ready ? "#00796b" : "#666666";
  std::ofstream o(path);
  o << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"320\" height=\"240\">";
  o << "<rect width=\"320\" height=\"240\" fill=\"#101418\"/>";
  o << "<rect width=\"320\" height=\"38\" fill=\"" << state_color << "\"/>";
  o << "<g fill=\"white\" font-family=\"monospace\">";
  o << "<text x=\"10\" y=\"25\" font-size=\"18\">CoreS3 BUS SIM</text>";
  o << "<text x=\"305\" y=\"25\" text-anchor=\"end\" font-size=\"16\">"
    << App::stateName(frame.state) << "</text>";
  o << "<text x=\"12\" y=\"62\" font-size=\"14\">" << xml(frame.message) << "</text>";
  if (!frame.warning.empty())
    o << "<text x=\"12\" y=\"82\" fill=\"#ffca28\" font-size=\"13\">WARN: "
      << xml(frame.warning) << "</text>";
  o << "<text x=\"12\" y=\"105\" font-size=\"13\">I2C:" << health(frame.i2c_ok && frame.imu_ok)
    << "  UART:" << health(frame.uart_ok) << "  GNSS:" << health(frame.gnss_ok) << "</text>";
  o << "<text x=\"12\" y=\"130\" font-size=\"13\">Pitch: " << std::fixed << std::setprecision(1)
    << frame.pitch_deg << " deg  Bat: " << std::setprecision(2) << frame.battery_v << " V</text>";
  o << "<text x=\"12\" y=\"153\" font-size=\"12\">ticks:" << frame.run_ticks << " t:"
    << t_ms << "ms GNSS-age:";
  if (sensor.gnss_age_ms == std::numeric_limits<std::uint32_t>::max()) o << "never";
  else o << sensor.gnss_age_ms << "ms";
  o << "</text>";
  o << "<rect x=\"60\" y=\"175\" width=\"200\" height=\"55\" rx=\"8\" fill=\"#37474f\"/>";
  o << "<text x=\"160\" y=\"210\" text-anchor=\"middle\" font-size=\"20\">"
    << xml(frame.button_label) << "</text></g></svg>";
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
  trace << "t_ms,state,pitch_deg,i2c_ok,imu_ok,uart_ok,gnss_ok,gnss_age_ms,run_ticks,warning,i2c_timeouts,uart_dropped\n";

  for (std::size_t i = 0; i < rows.size(); ++i) {
    const auto& r = rows[i];
    hw.apply(r.t_ms, r.hw);
    const auto sensor = hw.sample(r.t_ms);
    const auto frame = app.update(r.t_ms, sensor, r.touch);
    std::ostringstream filename;
    filename << "frame_" << std::setw(3) << std::setfill('0') << i << '_' << r.t_ms << "ms.svg";
    writeSvg(outdir / filename.str(), frame, sensor, r.t_ms);
    const auto& stats = hw.stats();
    trace << r.t_ms << ',' << App::stateName(frame.state) << ',' << sensor.pitch_deg << ','
          << sensor.i2c_ok << ',' << sensor.imu_ok << ',' << sensor.uart_ok << ',' << sensor.gnss_ok << ','
          << sensor.gnss_age_ms << ',' << frame.run_ticks << ",\"" << frame.warning << "\"," << stats.i2c_timeouts
          << ',' << stats.uart_frames_dropped << '\n';
    std::cout << r.t_ms << "ms " << App::stateName(frame.state)
              << " I2C=" << health(sensor.i2c_ok) << " UART=" << health(sensor.uart_ok)
              << " GNSS=" << health(sensor.gnss_ok) << '\n';
  }
  return 0;
}
