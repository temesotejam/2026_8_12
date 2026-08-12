#pragma once

#include <cstdint>
#include <string>

namespace cores3sim {

enum class RunState {
  Boot,
  Ready,
  Running,
  Fault,
};

struct SensorSample {
  // Keep the first three fields compatible with the phase-1 aggregate initializer.
  bool imu_ok{true};
  float pitch_deg{0.0f};
  float battery_v{4.0f};

  bool i2c_ok{true};
  bool uart_ok{true};
  bool gnss_ok{true};
  std::uint32_t gnss_age_ms{0};

  // Phase 3 transport/logging/timing health. Appended for source compatibility.
  bool i2c_nack{false};
  bool uart_frame_ok{true};
  bool timing_ok{true};
  std::int32_t loop_jitter_ms{0};
  bool sd_ok{true};
};

struct TouchSample {
  bool pressed{false};
  int x{0};
  int y{0};
};

struct UiFrame {
  RunState state{RunState::Boot};
  float pitch_deg{0.0f};
  float battery_v{0.0f};
  std::uint32_t run_ticks{0};
  bool button_enabled{false};
  bool i2c_ok{true};
  bool imu_ok{true};
  bool uart_ok{true};
  bool gnss_ok{true};
  std::uint32_t gnss_age_ms{0};
  bool i2c_nack{false};
  bool uart_frame_ok{true};
  bool timing_ok{true};
  std::int32_t loop_jitter_ms{0};
  bool sd_ok{true};
  std::string button_label{"WAIT"};
  std::string message{"Booting"};
  std::string warning{};
};

class App {
 public:
  App() = default;

  UiFrame update(std::uint32_t now_ms,
                 const SensorSample& sensors,
                 const TouchSample& touch);

  RunState state() const { return state_; }
  std::uint32_t runTicks() const { return run_ticks_; }

  static const char* stateName(RunState state);
  static bool inMainButton(int x, int y);

 private:
  RunState state_{RunState::Boot};
  bool previous_touch_pressed_{false};
  std::uint32_t recovery_started_ms_{0};
  std::uint32_t last_run_tick_ms_{0};
  std::uint32_t run_ticks_{0};
};

}  // namespace cores3sim
