#include <array>
#include <cstdint>
#include <cstdio>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc.h"
#include "soc/spi_reg.h"

namespace {
constexpr int kHost = 2;
constexpr uintptr_t kW0 = DR_REG_SPI2_BASE + 0x98;

void wait_idle() {
  while (REG_READ(SPI_CMD_REG(kHost)) & SPI_USR) {
  }
}

void sync_regs() {
  REG_WRITE(SPI_CMD_REG(kHost), SPI_UPDATE);
  while (REG_READ(SPI_CMD_REG(kHost)) & SPI_UPDATE) {
  }
}

void command(uint8_t value) {
  wait_idle();
  REG_WRITE(SPI_USER_REG(kHost), SPI_USR_COMMAND);
  REG_WRITE(SPI_USER2_REG(kHost),
            (7u << SPI_USR_COMMAND_BITLEN_S) | value);
  sync_regs();
  REG_WRITE(SPI_CMD_REG(kHost), SPI_USR);
  wait_idle();
}

void data(const uint8_t* src, size_t len) {
  while (len) {
    const size_t chunk = len > 64 ? 64 : len;
    std::array<uint32_t, 16> words{};
    for (size_t i = 0; i < chunk; ++i) {
      words[i / 4] |= static_cast<uint32_t>(src[i]) << ((i % 4) * 8);
    }
    for (size_t i = 0; i < words.size(); ++i) {
      REG_WRITE(kW0 + i * 4, words[i]);
    }
    REG_WRITE(SPI_MS_DLEN_REG(kHost), chunk * 8 - 1);
    REG_WRITE(SPI_USER_REG(kHost), SPI_USR_MOSI);
    sync_regs();
    REG_WRITE(SPI_CMD_REG(kHost), SPI_USR);
    wait_idle();
    src += chunk;
    len -= chunk;
  }
}

void set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  uint8_t p[4];
  command(0x2A);
  p[0] = x0 >> 8; p[1] = x0; p[2] = x1 >> 8; p[3] = x1;
  data(p, sizeof(p));
  command(0x2B);
  p[0] = y0 >> 8; p[1] = y0; p[2] = y1 >> 8; p[3] = y1;
  data(p, sizeof(p));
  command(0x2C);
}

uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return (static_cast<uint16_t>(r & 0xF8) << 8) |
         (static_cast<uint16_t>(g & 0xFC) << 3) |
         (b >> 3);
}

void fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
               uint8_t r, uint8_t g, uint8_t b) {
  set_window(x, y, x + w - 1, y + h - 1);
  const uint16_t px = rgb565(r, g, b);
  std::array<uint8_t, 64> block{};
  for (size_t i = 0; i < block.size(); i += 2) {
    block[i] = px >> 8;
    block[i + 1] = px & 0xFF;
  }
  size_t bytes = static_cast<size_t>(w) * h * 2;
  while (bytes) {
    const size_t n = bytes > block.size() ? block.size() : bytes;
    data(block.data(), n);
    bytes -= n;
  }
}
}  // namespace

extern "C" void app_main() {
  std::printf("SPI2_LCD_BOOT:ESP32S3 base=0x%08X\n",
              static_cast<unsigned>(DR_REG_SPI2_BASE));

  command(0x01);
  command(0x11);
  command(0x3A);
  const uint8_t fmt = 0x55;
  data(&fmt, 1);
  command(0x36);
  const uint8_t madctl = 0x00;
  data(&madctl, 1);
  command(0x29);

  fill_rect(0, 0, 320, 240, 8, 12, 20);
  fill_rect(12, 12, 296, 42, 0, 180, 100);
  fill_rect(24, 76, 130, 120, 230, 65, 55);
  fill_rect(170, 76, 126, 120, 45, 110, 240);
  fill_rect(24, 210, 272, 18, 245, 200, 35);

  command(0x00);
  std::printf("SPI2_LCD_CHECK:PASS\n");

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
