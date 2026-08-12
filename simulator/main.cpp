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

static void parsePhase4Base(const std::vector<std::string>& c,
                            ScenarioRow& r) {
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
}

static void parsePhase5Base(const std::vector<std::string>& c,
                            ScenarioRow& r) {
  parsePhase4Base(c, r);
  r.hw.ina_model_enabled = std::stoi(c[27]) != 0;
  r.hw.ina_powered = std::stoi(c[28]) != 0;
  r.hw.ina_device_ack = std::stoi(c[29]) != 0;
  r.hw.ina_force_reset = std::stoi(c[30]) != 0;
  r.hw.ina_stall_conversions = std::stoi(c[31]) != 0;
  r.hw.ina_calibration_programmed = std::stoi(c[32]) != 0;
  r.hw.ina_force_math_overflow = std::stoi(c[33]) != 0;
  r.hw.ina_bus_voltage_v = std::stof(c[34]);
  r.hw.ina_current_a = std::stof(c[35]);
  const auto conversion_us =
      static_cast<std::uint32_t>(std::stoul(c[36]));
  r.hw.ina_shunt_conversion_us = conversion_us;
  r.hw.ina_bus_conversion_us = conversion_us;
  r.hw.ina_averages = static_cast<std::uint32_t>(std::stoul(c[37]));
  r.hw.ina_reinit_delay_ms =
      static_cast<std::uint32_t>(std::stoul(c[38]));
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
      parsePhase4Base(c, r);
    } else if (c.size() == 39) {
      parsePhase5Base(c, r);
    } else if (c.size() == 49) {
      parsePhase5Base(c, r);
      r.hw.tof_model_enabled = std::stoi(c[39]) != 0;
      r.hw.tof_powered = std::stoi(c[40]) != 0;
      r.hw.tof_device_ack = std::stoi(c[41]) != 0;
      r.hw.tof_force_reset = std::stoi(c[42]) != 0;
      r.hw.tof_ranging_enabled = std::stoi(c[43]) != 0;
      r.hw.tof_stall_frames = std::stoi(c[44]) != 0;
      r.hw.tof_ranging_frequency_hz =
          static_cast<std::uint32_t>(std::stoul(c[45]));
      r.hw.tof_default_distance_mm =
          static_cast<std::uint16_t>(std::stoul(c[46]));
      r.hw.tof_invalid_zone = static_cast<std::uint8_t>(std::stoul(c[47]));
      r.hw.tof_invalid_status = static_cast<std::uint8_t>(std::stoul(c[48]));
    } else {
      throw std::runtime_error(
          "scenario row needs 7, 12, 20, 27, 39, or 49 columns: " + line);
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

static const char* inaState(const UiFrame& f) {
  if (!f.ina_model_active) return "N/A";
  if (!f.ina_device_ok) return "OFF";
  if (f.ina_reinitializing) return "INIT";
  if (!f.ina_initialized) return "OFF";
  if (!f.ina_calibration_ok) return "CAL";
  if (!f.ina_range_ok) return "RANGE";
  if (f.ina_math_overflow) return "OVF";
  if (!f.ina_conversion_fresh) return "STALE";
  return "RUN";
}

static const char* tofState(const UiFrame& f) {
  if (!f.tof_model_active) return "N/A";
  if (!f.tof_device_ok) return "OFF";
  if (f.tof_reinitializing) return "INIT";
  if (!f.tof_initialized) return "OFF";
  if (!f.tof_frame_fresh) return "STALE";
  if (f.tof_valid_zones < 64) return "PART";
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
  o << "<text x=\"9\" y=\"24\" font-size=\"16\">CoreS3 DEVICE SIM</text>";
  o << "<text x=\"309\" y=\"24\" text-anchor=\"end\" font-size=\"15\">"
    << App::stateName(f.state) << "</text>";
  o << "<text x=\"10\" y=\"54\" font-size=\"11\">" << xml(f.message) << "</text>";
  if (!f.warning.empty()) {
    o << "<text x=\"10\" y=\"70\" fill=\"#ffca28\" font-size=\"8\">WARN: "
      << xml(f.warning) << "</text>";
  }
  o << "<text x=\"10\" y=\"88\" font-size=\"9\">I2C:" << health(f.i2c_ok)
    << " BNO:" << bnoState(f) << " INA:" << inaState(f)
    << " TOF:" << tofState(f) << "</text>";
  o << "<text x=\"10\" y=\"104\" font-size=\"9\">TOF zones:"
    << static_cast<int>(f.tof_valid_zones) << "/64 min:"
    << f.tof_min_distance_mm << "mm age:";
  if (!f.tof_model_active) o << "n/a";
  else if (s.tof_frame_age_ms == std::numeric_limits<std::uint32_t>::max()) o << "never";
  else o << s.tof_frame_age_ms << "ms";
  o << "</text>";
  o << "<text x=\"10\" y=\"120\" font-size=\"9\">INA " << std::fixed
    << std::setprecision(2) << f.ina_bus_voltage_v << "V "
    << std::setprecision(3) << f.ina_current_a << "A "
    << std::setprecision(2) << f.ina_power_w << "W</text>";
  o << "<text x=\"10\" y=\"136\" font-size=\"9\">GNSS:"
    << health(f.gnss_ok) << " UART:" << health(f.uart_ok)
    << " SD:" << health(f.sd_ok) << " TIME:" << health(f.timing_ok) << "</text>";
  o << "<text x=\"10\" y=\"152\" font-size=\"9\">Pitch:"
    << std::setprecision(1) << f.pitch_deg << "deg t:" << t_ms
    << " ticks:" << f.run_ticks << "</text>";
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
  trace << "t_ms,state,pitch_deg,i2c_ok,imu_ok,uart_ok,gnss_ok,sd_ok,"
           "bno_active,bno_initialized,bno_fresh,bno_age_ms,"
           "ina_active,ina_device_ok,ina_initialized,ina_fresh,ina_bus_v,ina_current_a,ina_power_w,"
           "tof_active,tof_device_ok,tof_initialized,tof_reinitializing,tof_fresh,tof_age_ms,tof_valid_zones,tof_min_mm,"
           "run_ticks,warning,tof_resets,tof_reinit_attempts,tof_reinit_successes,tof_frames,tof_stale_events,tof_device_nacks,tof_invalid_zone_events\n";

  for (std::size_t i = 0; i < rows.size(); ++i) {
    const auto& r = rows[i];
    hw.apply(r.t_ms, r.hw);
    const auto sensor = hw.sample(r.t_ms);
    const auto frame = app.update(r.t_ms, sensor, r.touch);

    std::ostringstream filename;
    filename << "frame_" << std::setw(3) << std::setfill('0') << i
             << '_' << r.t_ms << "ms.svg";
    writeSvg(outdir / filename.str(), frame, sensor, r.t_ms);

    const auto& ts = hw.tofStats();
    trace << r.t_ms << ',' << App::stateName(frame.state) << ','
          << sensor.pitch_deg << ',' << sensor.i2c_ok << ',' << sensor.imu_ok << ','
          << sensor.uart_ok << ',' << sensor.gnss_ok << ',' << sensor.sd_ok << ','
          << sensor.bno_model_active << ',' << sensor.bno_initialized << ','
          << sensor.bno_report_fresh << ',' << sensor.bno_report_age_ms << ','
          << sensor.ina_model_active << ',' << sensor.ina_device_ok << ','
          << sensor.ina_initialized << ',' << sensor.ina_conversion_fresh << ','
          << sensor.ina_bus_voltage_v << ',' << sensor.ina_current_a << ','
          << sensor.ina_power_w << ',' << sensor.tof_model_active << ','
          << sensor.tof_device_ok << ',' << sensor.tof_initialized << ','
          << sensor.tof_reinitializing << ',' << sensor.tof_frame_fresh << ','
          << sensor.tof_frame_age_ms << ',' << static_cast<int>(sensor.tof_valid_zones)
          << ',' << sensor.tof_min_distance_mm << ',' << frame.run_ticks
          << ",\"" << frame.warning << "\"," << ts.resets << ','
          << ts.reinit_attempts << ',' << ts.reinit_successes << ','
          << ts.frames_delivered << ',' << ts.stale_events << ','
          << ts.device_nacks << ',' << ts.invalid_zone_events << '\n';

    std::cout << r.t_ms << "ms " << App::stateName(frame.state)
              << " BNO=" << bnoState(frame)
              << " INA=" << inaState(frame)
              << " TOF=" << tofState(frame)
              << " zones=" << static_cast<int>(frame.tof_valid_zones) << '\n';
  }

  return 0;
}
