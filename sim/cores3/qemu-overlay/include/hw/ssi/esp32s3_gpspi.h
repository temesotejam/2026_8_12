#pragma once

#include "hw/hw.h"
#include "hw/ssi/ssi.h"

#define TYPE_ESP32S3_GPSPI "ssi.esp32s3.gpspi"
#define ESP32S3_GPSPI(obj) OBJECT_CHECK(ESP32S3GpspiState, (obj), TYPE_ESP32S3_GPSPI)

#define ESP32S3_GPSPI_IO_SIZE 0x1000
#define ESP32S3_GPSPI_BUF_WORDS 16
#define ESP32S3_GPSPI_CS_COUNT 1

typedef struct ESP32S3GpspiState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    SSIBus *spi;
    qemu_irq cs_gpio[ESP32S3_GPSPI_CS_COUNT];
    qemu_irq dc_gpio;

    uint32_t cmd;
    uint32_t addr;
    uint32_t ctrl;
    uint32_t clock;
    uint32_t user;
    uint32_t user1;
    uint32_t user2;
    uint32_t ms_dlen;
    uint32_t misc;
    uint32_t data_reg[ESP32S3_GPSPI_BUF_WORDS];
} ESP32S3GpspiState;
