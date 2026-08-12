#include <Arduino.h>
#include <M5Unified.h>

#include "smoke_logic.hpp"

namespace {
constexpr uint32_t kHeartbeatPeriodMs = 1000;

uint32_t lastHeartbeatMs = 0;
bool displayReady = false;
bool resultReported = false;
smoke::HeartbeatGate heartbeatGate{3};
}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);

  auto cfg = M5.config();
  M5.begin(cfg);

  M5.Display.setRotation(1);
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(8, 8);
  M5.Display.println("CoreS3 / virtual CI");
  M5.Display.println("Smoke test");

  const int width = M5.Display.width();
  const int height = M5.Display.height();
  displayReady = width > 0 && height > 0;

  Serial.println("BOOT:CORE_S3_SMOKE_V2");
  Serial.printf("DISPLAY:%dx%d\n", width, height);

  if (!displayReady) {
    Serial.println("SIM_CHECK:FAIL display-not-ready");
    resultReported = true;
  }

  lastHeartbeatMs = millis();
}

void loop() {
  M5.update();

  const uint32_t now = millis();
  if (now - lastHeartbeatMs < kHeartbeatPeriodMs) {
    delay(1);
    return;
  }

  lastHeartbeatMs += kHeartbeatPeriodMs;
  const auto heartbeat = heartbeatGate.tick();

  Serial.printf("HEARTBEAT:%lu\n", static_cast<unsigned long>(heartbeat.count));

  M5.Display.setCursor(8, 70);
  M5.Display.printf("Heartbeat: %lu   ", static_cast<unsigned long>(heartbeat.count));
  M5.Display.setCursor(8, 95);
  M5.Display.printf("Uptime: %lu s   ", static_cast<unsigned long>(now / 1000));

  if (!resultReported && displayReady && heartbeat.just_passed) {
    Serial.println("SIM_CHECK:PASS");
    M5.Display.setCursor(8, 130);
    M5.Display.setTextColor(TFT_CYAN, TFT_BLACK);
    M5.Display.println("SIM_CHECK: PASS");
    resultReported = true;
  }
}
