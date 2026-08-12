#include <cstdio>

#include "esp_chip_info.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "smoke_logic.hpp"

extern "C" void app_main() {
  esp_chip_info_t chip_info{};
  esp_chip_info(&chip_info);

  std::printf("QEMU_BOOT:ESP32S3 cores=%d revision=%d\n",
              chip_info.cores,
              chip_info.revision);
  std::fflush(stdout);

  smoke::HeartbeatGate heartbeat_gate{3};

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    const auto heartbeat = heartbeat_gate.tick();

    std::printf("QEMU_HEARTBEAT:%lu\n",
                static_cast<unsigned long>(heartbeat.count));
    if (heartbeat.just_passed) {
      std::printf("QEMU_CHECK:PASS\n");
    }
    std::fflush(stdout);
  }
}
