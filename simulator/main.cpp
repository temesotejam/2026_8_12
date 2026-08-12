#include "App.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using cores3sim::App;
using cores3sim::SensorSample;
using cores3sim::TouchSample;
using cores3sim::UiFrame;
namespace fs = std::filesystem;

struct ScenarioRow {
  std::uint32_t t_ms{};
  SensorSample sensor{};
  TouchSample touch{};
};

static std::vector<std::string> split(const std::string& line, char delim) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, delim)) out.push_back(item);
  return out;
}

static std::vector<ScenarioRow> loadScenario(const fs::path& path) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("cannot open scenario: " + path.string());
  std::vector<ScenarioRow> rows;
  std::string line;
  bool first = true;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') continue;
    if (first) {
      first = false;
      if (line.rfind("t_ms,", 0) == 0) continue;
    }
    auto f = split(line, ',');
    if (f.size() != 7) throw std::runtime_error("scenario row needs 7 columns: " + line);
    ScenarioRow r;
    r.t_ms = static_cast<std::uint32_t>(std::stoul(f[0]));
    r.sensor.pitch_deg = std::stof(f[1]);
    r.sensor.battery_v = std::stof(f[2]);
    r.sensor.imu_ok = std::stoi(f[3]) != 0;
    r.touch.pressed = std::stoi(f[4]) != 0;
    r.touch.x = std::stoi(f[5]);
    r.touch.y = std::stoi(f[6]);
    rows.push_back(r);
  }
  return rows;
}

static std::string escapeXml(const std::string& s) {
  std::string out;
  for (char c : s) {
    switch (c) {
      case '&': out += "&amp;"; break;
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '"': out += "&quot;"; break;
      default: out += c; break;
    }
  }
  return out;
}

static void writeSvg(const fs::path& path, const UiFrame& f, std::uint32_t t_ms) {
  const char* statusColor = "#2b2b2b";
  switch (f.state) {
    case cores3sim::RunState::Boot: statusColor = "#6d6d6d"; break;
    case cores3sim::RunState::Ready: statusColor = "#00796b"; break;
    case cores3sim::RunState::Running: statusColor = "#1565c0"; break;
    case cores3sim::RunState::Fault: statusColor = "#c62828"; break;
  }
  const double gaugeX = 160.0 + (f.pitch_deg / 45.0) * 120.0;
  const double clampedX = gaugeX < 40 ? 40 : (gaugeX > 280 ? 280 : gaugeX);

  std::ofstream o(path);
  o << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"320\" height=\"240\" viewBox=\"0 0 320 240\">\n";
  o << "<rect width=\"320\" height=\"240\" fill=\"#101418\"/>\n";
  o << "<rect x=\"0\" y=\"0\" width=\"320\" height=\"38\" fill=\"" << statusColor << "\"/>\n";
  o << "<text x=\"12\" y=\"25\" fill=\"white\" font-family=\"monospace\" font-size=\"18\">CoreS3 SIM</text>\n";
  o << "<text x=\"308\" y=\"25\" fill=\"white\" text-anchor=\"end\" font-family=\"monospace\" font-size=\"16\">" << App::stateName(f.state) << "</text>\n";
  o << "<text x=\"15\" y=\"67\" fill=\"#e8eef2\" font-family=\"monospace\" font-size=\"15\">" << escapeXml(f.message) << "</text>\n";
  o << "<text x=\"15\" y=\"94\" fill=\"#e8eef2\" font-family=\"monospace\" font-size=\"14\">Pitch: " << std::fixed << std::setprecision(1) << f.pitch_deg << " deg</text>\n";
  o << "<text x=\"210\" y=\"94\" fill=\"#e8eef2\" font-family=\"monospace\" font-size=\"14\">Bat: " << std::fixed << std::setprecision(2) << f.battery_v << " V</text>\n";
  o << "<line x1=\"40\" y1=\"125\" x2=\"280\" y2=\"125\" stroke=\"#73808a\" stroke-width=\"4\"/>\n";
  o << "<line x1=\"160\" y1=\"114\" x2=\"160\" y2=\"136\" stroke=\"#ffffff\" stroke-width=\"2\"/>\n";
  o << "<circle cx=\"" << clampedX << "\" cy=\"125\" r=\"8\" fill=\"#ffca28\"/>\n";
  o << "<text x=\"15\" y=\"157\" fill=\"#b8c4cc\" font-family=\"monospace\" font-size=\"13\">run ticks: " << f.run_ticks << "   t=" << t_ms << "ms</text>\n";
  o << "<rect x=\"60\" y=\"175\" width=\"200\" height=\"55\" rx=\"8\" fill=\"" << (f.button_enabled ? "#37474f" : "#252a2e") << "\" stroke=\"#90a4ae\"/>\n";
  o << "<text x=\"160\" y=\"209\" fill=\"white\" text-anchor=\"middle\" font-family=\"monospace\" font-size=\"20\">" << escapeXml(f.button_label) << "</text>\n";
  o << "</svg>\n";
}

int main(int argc, char** argv) {
  const fs::path scenario = argc > 1 ? argv[1] : "scenarios/demo.csv";
  const fs::path outdir = argc > 2 ? argv[2] : "artifacts";
  fs::create_directories(outdir);
  auto rows = loadScenario(scenario);
  if (rows.empty()) throw std::runtime_error("scenario is empty");

  App app;
  std::ofstream trace(outdir / "trace.csv");
  trace << "t_ms,state,pitch_deg,battery_v,run_ticks,message\n";

  for (std::size_t i = 0; i < rows.size(); ++i) {
    const auto& r = rows[i];
    UiFrame frame = app.update(r.t_ms, r.sensor, r.touch);
    std::ostringstream name;
    name << "frame_" << std::setw(3) << std::setfill('0') << i << "_" << r.t_ms << "ms.svg";
    writeSvg(outdir / name.str(), frame, r.t_ms);
    trace << r.t_ms << ',' << App::stateName(frame.state) << ',' << frame.pitch_deg << ',' << frame.battery_v << ',' << frame.run_ticks << ',' << '"' << frame.message << '"' << '\n';
    std::cout << std::setw(5) << r.t_ms << " ms  " << std::setw(7) << App::stateName(frame.state) << "  pitch=" << std::fixed << std::setprecision(1) << frame.pitch_deg << "  ticks=" << frame.run_ticks << '\n';
  }
  std::cout << "Wrote " << rows.size() << " SVG frames and trace.csv to " << outdir << '\n';
  return 0;
}
