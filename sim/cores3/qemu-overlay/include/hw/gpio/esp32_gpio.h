#pragma once

#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/registerfields.h"

#define TYPE_ESP32_GPIO "esp32.gpio"
#define ESP32_GPIO(obj)             OBJECT_CHECK(Esp32GpioState, (obj), TYPE_ESP32_GPIO)
#define ESP32_GPIO_GET_CLASS(obj)   OBJECT_GET_CLASS(Esp32GpioClass, obj, TYPE_ESP32_GPIO)
#define ESP32_GPIO_CLASS(klass)     OBJECT_CLASS_CHECK(Esp32GpioClass, klass, TYPE_ESP32_GPIO)

REG32(GPIO_STRAP, 0x0038)

#define ESP32_STRAP_MODE_FLASH_BOOT 0x12
#define ESP32_STRAP_MODE_UART_BOOT  0x0f
#define ESP32_GPIO_PIN_COUNT 54

typedef struct Esp32GpioState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t strap_mode;

    /* Minimal digital GPIO state required by the CoreS3 virtual board.
     * The real ESP32-S3 has 0..48 usable GPIOs; 54 slots keeps this parent
     * compatible with the wider ESP32 family while remaining inexpensive. */
    uint32_t out_reg;
    uint32_t out1_reg;
    uint32_t enable_reg;
    uint32_t enable1_reg;
    uint32_t external_in_reg;
    uint32_t external_in1_reg;
    qemu_irq pin_out[ESP32_GPIO_PIN_COUNT];
} Esp32GpioState;

typedef struct Esp32GpioClass {
    SysBusDeviceClass parent_class;
} Esp32GpioClass;
