#include <Arduino.h>
#include <M5Unified.h>

#include "smoke_logic.hpp"

namespace {
constexpr uint32_t kHeartbeatPeriodMs = 1000;

uint32_t lastHeartbeatMs = 0;
bool displayReady = false;
bool resultReported = false;
smoke::HeartbeatGate heartbeatGate{3};

void probeStage(const char* stage) {
  Serial0.print("CORES3_QEMU_PROBE:");
  Serial0.println(stage);
}
}  // namespace

void setup() {
  // CoreS3 normally maps Arduino `Serial` to USB CDC. Keep a second, explicit
  // UART0 probe so Espressif QEMU can show exactly how far the real firmware gets.
  Serial0.begin(115200);
  delay(20);
  probeStage("SERIAL0_READY");

  Serial.begin(115200);
  delay(200);
  probeStage("ARDUINO_SERIAL_READY");

  // M5GFX only enters the ESP32-S3/CoreS3 autodetection path for the expected
  // ESP32-S3 package value. Expose it over UART0 so the virtual-board CI can
  // distinguish an eFuse/package-identification problem from an I2C problem.
  Serial0.printf("CORES3_QEMU_PROBE:PKG_VER=%lu\n",
                 static_cast<unsigned long>(m5gfx::get_pkg_ver()));

  auto cfg = M5.config();
  probeStage("M5_BEGIN_ENTER");
  M5.begin(cfg);
  probeStage("M5_BEGIN_EXIT");
  Serial0.printf("CORES3_QEMU_PROBE:BOARD=%u\n",
                 static_cast<unsigned>(M5.getBoard()));

  probeStage("DISPLAY_DRAW_ENTER");
  M5.Display.setRotation(1);
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(8, 8);
  M5.Display.println("CoreS3 / virtual CI");
  M5.Display.println("Smoke test");
  probeStage("DISPLAY_DRAW_EXIT");

  const int width = M5.Display.width();
  const int height = M5.Display.height();
  displayReady = width > 0 && height > 0;

  Serial.println("BOOT:CORE_S3_SMOKE_V2");
  Serial.printf("DISPLAY:%dx%d\n", width, height);
  Serial0.printf("CORES3_QEMU_PROBE:DISPLAY=%dx%d\n", width, height);

  if (!displayReady) {
    Serial.println("SIM_CHECK:FAIL display-not-ready");
    Serial0.println("CORES3_QEMU_PROBE:DISPLAY_NOT_READY");
    resultReported = true;
  }

  lastHeartbeatMs = millis();
  probeStage("SETUP_EXIT");
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
  Serial0.printf("CORES3_QEMU_PROBE:HEARTBEAT=%lu\n",
                 static_cast<unsigned long>(heartbeat.count));

  M5.Display.setCursor(8, 70);
  M5.Display.printf("Heartbeat: %lu   ", static_cast<unsigned long>(heartbeat.count));
  M5.Display.setCursor(8, 95);
  M5.Display.printf("Uptime: %lu s   ", static_cast<unsigned long>(now / 1000));

  if (!resultReported && displayReady && heartbeat.just_passed) {
    Serial.println("SIM_CHECK:PASS");
    Serial0.println("CORES3_QEMU_PROBE:SIM_CHECK_PASS");
    M5.Display.setCursor(8, 130);
    M5.Display.setTextColor(TFT_CYAN, TFT_BLACK);
    M5.Display.println("SIM_CHECK: PASS");
    resultReported = true;
  }
}