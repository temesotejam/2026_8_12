/*
 * Minimal ESP32-S3 general-purpose SPI (SPI2) controller model.
 *
 * This is intentionally a CPU-buffer/polling subset for the CoreS3 virtual
 * hardware experiment. It models the register path needed to execute a user
 * transaction through an SSI bus. DMA, interrupts and timing are future work.
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/ssi/esp32s3_gpspi.h"
#include "qemu/module.h"

#define GPSPI_CMD       0x000
#define GPSPI_ADDR      0x004
#define GPSPI_CTRL      0x008
#define GPSPI_CLOCK     0x00c
#define GPSPI_USER      0x010
#define GPSPI_USER1     0x014
#define GPSPI_USER2     0x018
#define GPSPI_MS_DLEN   0x01c
#define GPSPI_MISC      0x020
#define GPSPI_W0        0x098
#define GPSPI_W15       0x0d4

#define GPSPI_CMD_UPDATE     (1u << 23)
#define GPSPI_CMD_USR        (1u << 24)
#define GPSPI_USER_USR_MOSI  (1u << 27)
#define GPSPI_USER_USR_MISO  (1u << 28)
#define GPSPI_USER_USR_DUMMY (1u << 29)
#define GPSPI_USER_USR_ADDR  (1u << 30)
#define GPSPI_USER_USR_CMD   (1u << 31)
#define GPSPI_MISC_KEEP_CS   (1u << 30)

static void gpspi_set_cs(ESP32S3GpspiState *s, bool active)
{
    qemu_set_irq(s->cs_gpio[0], active ? 0 : 1);
}

static void gpspi_set_dc(ESP32S3GpspiState *s, bool data)
{
    qemu_set_irq(s->dc_gpio, data ? 1 : 0);
}

static uint8_t gpspi_transfer_byte(ESP32S3GpspiState *s, uint8_t tx)
{
    return (uint8_t)ssi_transfer(s->spi, tx);
}

static void gpspi_transfer_bits_from_value(ESP32S3GpspiState *s,
                                           uint32_t value,
                                           unsigned bits,
                                           bool data)
{
    if (!bits) {
        return;
    }
    gpspi_set_dc(s, data);
    unsigned bytes = (bits + 7) / 8;
    for (int i = (int)bytes - 1; i >= 0; --i) {
        gpspi_transfer_byte(s, (value >> (i * 8)) & 0xff);
    }
}

static void gpspi_transfer_data(ESP32S3GpspiState *s, unsigned bits,
                                bool mosi, bool miso)
{
    if (!bits || (!mosi && !miso)) {
        return;
    }
    gpspi_set_dc(s, true);
    unsigned bytes = (bits + 7) / 8;
    if (bytes > ESP32S3_GPSPI_BUF_WORDS * 4) {
        bytes = ESP32S3_GPSPI_BUF_WORDS * 4;
    }

    for (unsigned i = 0; i < bytes; ++i) {
        unsigned word = i / 4;
        unsigned shift = (i % 4) * 8;
        uint8_t tx = mosi ? ((s->data_reg[word] >> shift) & 0xff) : 0;
        uint8_t rx = gpspi_transfer_byte(s, tx);
        if (miso) {
            s->data_reg[word] &= ~(0xffu << shift);
            s->data_reg[word] |= ((uint32_t)rx << shift);
        }
    }
}

static void gpspi_transaction(ESP32S3GpspiState *s)
{
    gpspi_set_cs(s, true);

    if (s->user & GPSPI_USER_USR_CMD) {
        unsigned bits = ((s->user2 >> 28) & 0xf) + 1;
        uint32_t value = s->user2 & 0xffff;
        gpspi_transfer_bits_from_value(s, value, bits, false);
    }

    if (s->user & GPSPI_USER_USR_ADDR) {
        unsigned bits = ((s->user1 >> 27) & 0x1f) + 1;
        gpspi_transfer_bits_from_value(s, s->addr, bits, true);
    }

    if (s->user & GPSPI_USER_USR_DUMMY) {
        unsigned cycles = (s->user1 & 0xff) + 1;
        gpspi_set_dc(s, true);
        for (unsigned i = 0; i < (cycles + 7) / 8; ++i) {
            gpspi_transfer_byte(s, 0);
        }
    }

    if ((s->user & GPSPI_USER_USR_MOSI) || (s->user & GPSPI_USER_USR_MISO)) {
        unsigned bits = (s->ms_dlen & 0x3ffff) + 1;
        gpspi_transfer_data(s, bits,
                            (s->user & GPSPI_USER_USR_MOSI) != 0,
                            (s->user & GPSPI_USER_USR_MISO) != 0);
    }

    if (!(s->misc & GPSPI_MISC_KEEP_CS)) {
        gpspi_set_cs(s, false);
    }

    s->cmd &= ~GPSPI_CMD_USR;
}

static uint64_t gpspi_read(void *opaque, hwaddr addr, unsigned size)
{
    ESP32S3GpspiState *s = ESP32S3_GPSPI(opaque);

    switch (addr) {
    case GPSPI_CMD: return s->cmd;
    case GPSPI_ADDR: return s->addr;
    case GPSPI_CTRL: return s->ctrl;
    case GPSPI_CLOCK: return s->clock;
    case GPSPI_USER: return s->user;
    case GPSPI_USER1: return s->user1;
    case GPSPI_USER2: return s->user2;
    case GPSPI_MS_DLEN: return s->ms_dlen;
    case GPSPI_MISC: return s->misc;
    default:
        if (addr >= GPSPI_W0 && addr <= GPSPI_W15 && !(addr & 3)) {
            return s->data_reg[(addr - GPSPI_W0) / 4];
        }
        return 0;
    }
}

static void gpspi_write(void *opaque, hwaddr addr, uint64_t value,
                        unsigned size)
{
    ESP32S3GpspiState *s = ESP32S3_GPSPI(opaque);
    uint32_t v = (uint32_t)value;

    switch (addr) {
    case GPSPI_CMD:
        s->cmd = v;
        if (v & GPSPI_CMD_UPDATE) {
            s->cmd &= ~GPSPI_CMD_UPDATE;
        }
        if (v & GPSPI_CMD_USR) {
            gpspi_transaction(s);
        }
        break;
    case GPSPI_ADDR: s->addr = v; break;
    case GPSPI_CTRL: s->ctrl = v; break;
    case GPSPI_CLOCK: s->clock = v; break;
    case GPSPI_USER: s->user = v; break;
    case GPSPI_USER1: s->user1 = v; break;
    case GPSPI_USER2: s->user2 = v; break;
    case GPSPI_MS_DLEN: s->ms_dlen = v; break;
    case GPSPI_MISC: s->misc = v; break;
    default:
        if (addr >= GPSPI_W0 && addr <= GPSPI_W15 && !(addr & 3)) {
            s->data_reg[(addr - GPSPI_W0) / 4] = v;
        }
        break;
    }
}

static const MemoryRegionOps gpspi_ops = {
    .read = gpspi_read,
    .write = gpspi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void gpspi_reset_hold(Object *obj, ResetType type)
{
    ESP32S3GpspiState *s = ESP32S3_GPSPI(obj);
    memset(s->data_reg, 0, sizeof(s->data_reg));
    s->cmd = 0;
    s->addr = 0;
    s->ctrl = 0;
    s->clock = 0x80003043;
    s->user = GPSPI_USER_USR_CMD;
    s->user1 = (23u << 27) | 7u;
    s->user2 = (7u << 28);
    s->ms_dlen = 0;
    s->misc = 0x3e;
    gpspi_set_cs(s, false);
    gpspi_set_dc(s, true);
}

static void gpspi_init(Object *obj)
{
    ESP32S3GpspiState *s = ESP32S3_GPSPI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &gpspi_ops, s,
                          TYPE_ESP32S3_GPSPI, ESP32S3_GPSPI_IO_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    s->spi = ssi_create_bus(DEVICE(s), "spi");
    qdev_init_gpio_out_named(DEVICE(s), &s->cs_gpio[0], SSI_GPIO_CS,
                             ESP32S3_GPSPI_CS_COUNT);
    qdev_init_gpio_out_named(DEVICE(s), &s->dc_gpio, "dc", 1);
}

static void gpspi_class_init(ObjectClass *klass, void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    rc->phases.hold = gpspi_reset_hold;
}

static const TypeInfo gpspi_info = {
    .name = TYPE_ESP32S3_GPSPI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3GpspiState),
    .instance_init = gpspi_init,
    .class_init = gpspi_class_init,
};

static void gpspi_register_types(void)
{
    type_register_static(&gpspi_info);
}

type_init(gpspi_register_types)
