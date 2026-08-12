#include <Arduino.h>
#include <M5Unified.h>
#include <cmath>

#include <App.h>

using cores3sim::App;
using cores3sim::SensorSample;
using cores3sim::TouchSample;
using cores3sim::UiFrame;

namespace {
App app;
SensorSample sensor;
std::uint32_t last_draw_ms = 0;

uint16_t stateColor(cores3sim::RunState state) {
  switch (state) {
    case cores3sim::RunState::Boot: return TFT_DARKGREY;
    case cores3sim::RunState::Ready: return TFT_DARKGREEN;
    case cores3sim::RunState::Running: return TFT_BLUE;
    case cores3sim::RunState::Fault: return TFT_RED;
  }
  return TFT_DARKGREY;
}

void readImu() {
  sensor.imu_ok = M5.Imu.getType() != m5::imu_none;
  if (!sensor.imu_ok) return;
  if (!M5.Imu.update()) return;
  const auto data = M5.Imu.getImuData();
  const float ax = data.accel.x;
  const float ay = data.accel.y;
  const float az = data.accel.z;
  sensor.pitch_deg = std::atan2(-ax, std::sqrt(ay * ay + az * az)) * 180.0f / PI;
}

TouchSample readTouch() {
  TouchSample out;
  if (M5.Touch.getCount() > 0) {
    const auto t = M5.Touch.getDetail(0);
    out.pressed = t.isPressed();
    out.x = t.x;
    out.y = t.y;
  }
  return out;
}

void draw(const UiFrame& frame) {
  auto& d = M5.Display;
  d.fillScreen(TFT_BLACK);
  d.fillRect(0, 0, 320, 38, stateColor(frame.state));
  d.setTextColor(TFT_WHITE);
  d.setTextSize(2);
  d.setCursor(10, 10);
  d.print("CoreS3 SIM DEMO");
  d.setCursor(235, 10);
  d.print(App::stateName(frame.state));
  d.setTextSize(2);
  d.setCursor(12, 52);
  d.print(frame.message.c_str());
  d.setCursor(12, 82);
  d.printf("Pitch: %6.1f deg", frame.pitch_deg);
  d.setCursor(12, 110);
  d.printf("Run ticks: %lu", static_cast<unsigned long>(frame.run_ticks));
  d.drawLine(40, 150, 280, 150, TFT_DARKGREY);
  d.drawLine(160, 140, 160, 160, TFT_WHITE);
  int marker_x = 160 + static_cast<int>((frame.pitch_deg / 45.0f) * 120.0f);
  if (marker_x < 40) marker_x = 40;
  if (marker_x > 280) marker_x = 280;
  d.fillCircle(marker_x, 150, 8, TFT_YELLOW);
  d.drawRoundRect(60, 175, 200, 55, 8, TFT_LIGHTGREY);
  d.setCursor(120, 194);
  d.print(frame.button_label.c_str());
}
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(1);
  Serial.begin(115200);
  sensor.battery_v = 4.0f;
  sensor.imu_ok = M5.Imu.getType() != m5::imu_none;
  Serial.printf("CoreS3 simulation demo boot. IMU=%s\n", sensor.imu_ok ? "OK" : "NONE");
}

void loop() {
  M5.update();
  readImu();
  const TouchSample touch = readTouch();
  const std::uint32_t now = millis();
  const UiFrame frame = app.update(now, sensor, touch);
  if (now - last_draw_ms >= 100) {
    last_draw_ms = now;
    draw(frame);
  }
  delay(5);
}
