/*
 * ESP32 GPIO emulation with digital line state for the CoreS3 experiment.
 *
 * This keeps Espressif QEMU's strap handling and adds the small subset of
 * GPIO OUT/ENABLE/IN semantics needed by M5GFX bit-banged I2C, manual LCD
 * CS/DC pins, and future external-port wiring.
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/gpio/esp32_gpio.h"

#define GPIO_OUT_OFS          0x04
#define GPIO_OUT_W1TS_OFS     0x08
#define GPIO_OUT_W1TC_OFS     0x0c
#define GPIO_OUT1_OFS         0x10
#define GPIO_OUT1_W1TS_OFS    0x14
#define GPIO_OUT1_W1TC_OFS    0x18
#define GPIO_ENABLE_OFS       0x20
#define GPIO_ENABLE_W1TS_OFS  0x24
#define GPIO_ENABLE_W1TC_OFS  0x28
#define GPIO_ENABLE1_OFS      0x2c
#define GPIO_ENABLE1_W1TS_OFS 0x30
#define GPIO_ENABLE1_W1TC_OFS 0x34
#define GPIO_IN_OFS           0x3c
#define GPIO_IN1_OFS          0x40

static bool pin_bit(uint32_t lo, uint32_t hi, unsigned pin)
{
    if (pin < 32) {
        return (lo >> pin) & 1u;
    }
    return (hi >> (pin - 32)) & 1u;
}

static int esp32_gpio_master_level(Esp32GpioState *s, unsigned pin)
{
    bool enabled = pin_bit(s->enable_reg, s->enable1_reg, pin);
    bool out = pin_bit(s->out_reg, s->out1_reg, pin);

    /* Treat a disabled output or an output-high as a released/high line.
     * This is exactly what the open-drain CoreS3 I2C probe needs. */
    return (enabled && !out) ? 0 : 1;
}

static void esp32_gpio_update_pin(Esp32GpioState *s, unsigned pin)
{
    if (pin < ESP32_GPIO_PIN_COUNT) {
        qemu_set_irq(s->pin_out[pin], esp32_gpio_master_level(s, pin));
    }
}

static void esp32_gpio_update_bank(Esp32GpioState *s, unsigned first,
                                   unsigned count)
{
    for (unsigned i = 0; i < count; ++i) {
        esp32_gpio_update_pin(s, first + i);
    }
}

static uint32_t esp32_gpio_input_low(Esp32GpioState *s)
{
    /* Open-drain/wired-AND view: an enabled low output wins; otherwise the
     * externally driven/pulled-up line is visible. */
    uint32_t driven_low = s->enable_reg & ~s->out_reg;
    return s->external_in_reg & ~driven_low;
}

static uint32_t esp32_gpio_input_high(Esp32GpioState *s)
{
    uint32_t driven_low = s->enable1_reg & ~s->out1_reg;
    return s->external_in1_reg & ~driven_low & 0x003fffffu;
}

static void esp32_gpio_set_input(void *opaque, int pin, int level)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);
    if (pin < 0 || pin >= ESP32_GPIO_PIN_COUNT) {
        return;
    }

    if (pin < 32) {
        uint32_t bit = 1u << pin;
        if (level) {
            s->external_in_reg |= bit;
        } else {
            s->external_in_reg &= ~bit;
        }
    } else {
        uint32_t bit = 1u << (pin - 32);
        if (level) {
            s->external_in1_reg |= bit;
        } else {
            s->external_in1_reg &= ~bit;
        }
    }
}

static uint64_t esp32_gpio_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);

    switch (addr) {
    case GPIO_OUT_OFS: return s->out_reg;
    case GPIO_OUT1_OFS: return s->out1_reg;
    case GPIO_ENABLE_OFS: return s->enable_reg;
    case GPIO_ENABLE1_OFS: return s->enable1_reg;
    case A_GPIO_STRAP: return s->strap_mode;
    case GPIO_IN_OFS: return esp32_gpio_input_low(s);
    case GPIO_IN1_OFS: return esp32_gpio_input_high(s);
    default: return 0;
    }
}

static void esp32_gpio_write(void *opaque, hwaddr addr,
                             uint64_t value, unsigned int size)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);
    uint32_t v = (uint32_t)value;

    switch (addr) {
    case GPIO_OUT_OFS:
        s->out_reg = v;
        esp32_gpio_update_bank(s, 0, 32);
        break;
    case GPIO_OUT_W1TS_OFS:
        s->out_reg |= v;
        esp32_gpio_update_bank(s, 0, 32);
        break;
    case GPIO_OUT_W1TC_OFS:
        s->out_reg &= ~v;
        esp32_gpio_update_bank(s, 0, 32);
        break;
    case GPIO_OUT1_OFS:
        s->out1_reg = v & 0x003fffffu;
        esp32_gpio_update_bank(s, 32, ESP32_GPIO_PIN_COUNT - 32);
        break;
    case GPIO_OUT1_W1TS_OFS:
        s->out1_reg |= v & 0x003fffffu;
        esp32_gpio_update_bank(s, 32, ESP32_GPIO_PIN_COUNT - 32);
        break;
    case GPIO_OUT1_W1TC_OFS:
        s->out1_reg &= ~(v & 0x003fffffu);
        esp32_gpio_update_bank(s, 32, ESP32_GPIO_PIN_COUNT - 32);
        break;
    case GPIO_ENABLE_OFS:
        s->enable_reg = v;
        esp32_gpio_update_bank(s, 0, 32);
        break;
    case GPIO_ENABLE_W1TS_OFS:
        s->enable_reg |= v;
        esp32_gpio_update_bank(s, 0, 32);
        break;
    case GPIO_ENABLE_W1TC_OFS:
        s->enable_reg &= ~v;
        esp32_gpio_update_bank(s, 0, 32);
        break;
    case GPIO_ENABLE1_OFS:
        s->enable1_reg = v & 0x003fffffu;
        esp32_gpio_update_bank(s, 32, ESP32_GPIO_PIN_COUNT - 32);
        break;
    case GPIO_ENABLE1_W1TS_OFS:
        s->enable1_reg |= v & 0x003fffffu;
        esp32_gpio_update_bank(s, 32, ESP32_GPIO_PIN_COUNT - 32);
        break;
    case GPIO_ENABLE1_W1TC_OFS:
        s->enable1_reg &= ~(v & 0x003fffffu);
        esp32_gpio_update_bank(s, 32, ESP32_GPIO_PIN_COUNT - 32);
        break;
    default:
        break;
    }
}

static const MemoryRegionOps gpio_ops = {
    .read = esp32_gpio_read,
    .write = esp32_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_gpio_reset_hold(Object *obj, ResetType type)
{
    Esp32GpioState *s = ESP32_GPIO(obj);
    s->out_reg = 0;
    s->out1_reg = 0;
    s->enable_reg = 0;
    s->enable1_reg = 0;
    /* Model board pull-ups / floating inputs as high by default. External
     * devices may pull individual inputs low through QEMU GPIO inputs. */
    s->external_in_reg = 0xffffffffu;
    s->external_in1_reg = 0x003fffffu;
    esp32_gpio_update_bank(s, 0, ESP32_GPIO_PIN_COUNT);
}

static void esp32_gpio_realize(DeviceState *dev, Error **errp)
{
}

static void esp32_gpio_init(Object *obj)
{
    Esp32GpioState *s = ESP32_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    DeviceState *dev = DEVICE(obj);

    object_property_set_int(obj, "strap_mode", ESP32_STRAP_MODE_FLASH_BOOT,
                            &error_fatal);

    memory_region_init_io(&s->iomem, obj, &gpio_ops, s,
                          TYPE_ESP32_GPIO, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_in(dev, esp32_gpio_set_input, ESP32_GPIO_PIN_COUNT);
    qdev_init_gpio_out(dev, s->pin_out, ESP32_GPIO_PIN_COUNT);

    s->external_in_reg = 0xffffffffu;
    s->external_in1_reg = 0x003fffffu;
}

static Property esp32_gpio_properties[] = {
    DEFINE_PROP_UINT32("strap_mode", Esp32GpioState, strap_mode, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_gpio_reset_hold;
    dc->realize = esp32_gpio_realize;
    device_class_set_props(dc, esp32_gpio_properties);
}

static const TypeInfo esp32_gpio_info = {
    .name = TYPE_ESP32_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32GpioState),
    .instance_init = esp32_gpio_init,
    .class_init = esp32_gpio_class_init,
    .class_size = sizeof(Esp32GpioClass),
};

static void esp32_gpio_register_types(void)
{
    type_register_static(&esp32_gpio_info);
}

type_init(esp32_gpio_register_types)
