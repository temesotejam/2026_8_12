/*
 * Minimal ILI9342C model for the M5Stack CoreS3 virtual-hardware experiment.
 *
 * Supported commands are deliberately focused on firmware-visible display
 * behavior: software reset, sleep out, display on, CASET/PASET, RAMWR,
 * MADCTL, COLMOD and RDDID. RGB565 writes update a 320x240 framebuffer.
 */

#include "qemu/osdep.h"
#include "hw/ssi/ssi.h"
#include "hw/qdev-properties.h"
#include "qemu/module.h"
#include "qemu/log.h"

#define TYPE_CORES3_ILI9342C "cores3-ili9342c"
OBJECT_DECLARE_SIMPLE_TYPE(CoreS3ILI9342CState, CORES3_ILI9342C)

#define LCD_W 320
#define LCD_H 240

#define CMD_SWRESET 0x01
#define CMD_RDDID   0x04
#define CMD_SLPOUT  0x11
#define CMD_DISPON  0x29
#define CMD_CASET   0x2a
#define CMD_PASET   0x2b
#define CMD_RAMWR   0x2c
#define CMD_MADCTL  0x36
#define CMD_COLMOD  0x3a

struct CoreS3ILI9342CState {
    SSIPeripheral parent_obj;
    bool dc;
    bool display_on;
    bool sleeping;
    bool dirty;
    uint8_t current_cmd;
    uint8_t pixel_format;
    uint8_t madctl;
    uint8_t pending[8];
    unsigned pending_len;
    unsigned read_index;
    uint16_t x0, x1, y0, y1;
    uint16_t x, y;
    bool pixel_high_valid;
    uint8_t pixel_high;
    uint8_t framebuffer[LCD_W * LCD_H * 3];
    char *output;
};

static void lcd_reset(CoreS3ILI9342CState *s)
{
    memset(s->framebuffer, 0, sizeof(s->framebuffer));
    s->display_on = false;
    s->sleeping = true;
    s->dirty = true;
    s->current_cmd = 0;
    s->pixel_format = 0x55;
    s->madctl = 0;
    s->pending_len = 0;
    s->read_index = 0;
    s->x0 = 0;
    s->x1 = LCD_W - 1;
    s->y0 = 0;
    s->y1 = LCD_H - 1;
    s->x = s->x0;
    s->y = s->y0;
    s->pixel_high_valid = false;
}

static void lcd_dump_ppm(CoreS3ILI9342CState *s)
{
    if (!s->output || !*s->output || !s->dirty) {
        return;
    }
    FILE *f = fopen(s->output, "wb");
    if (!f) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "CoreS3 ILI9342C: cannot open %s: %s\n",
                      s->output, strerror(errno));
        return;
    }
    fprintf(f, "P6\n%d %d\n255\n", LCD_W, LCD_H);
    fwrite(s->framebuffer, 1, sizeof(s->framebuffer), f);
    fclose(f);
    s->dirty = false;
}

static void lcd_advance(CoreS3ILI9342CState *s)
{
    s->x++;
    if (s->x > s->x1) {
        s->x = s->x0;
        s->y++;
        if (s->y > s->y1) {
            s->y = s->y0;
        }
    }
}

static void lcd_write_pixel565(CoreS3ILI9342CState *s, uint16_t raw)
{
    uint8_t r5 = (raw >> 11) & 0x1f;
    uint8_t g6 = (raw >> 5) & 0x3f;
    uint8_t b5 = raw & 0x1f;
    uint8_t r = (r5 * 255 + 15) / 31;
    uint8_t g = (g6 * 255 + 31) / 63;
    uint8_t b = (b5 * 255 + 15) / 31;
    if (s->x < LCD_W && s->y < LCD_H) {
        size_t pos = ((size_t)s->y * LCD_W + s->x) * 3;
        s->framebuffer[pos + 0] = r;
        s->framebuffer[pos + 1] = g;
        s->framebuffer[pos + 2] = b;
        s->dirty = true;
    }
    lcd_advance(s);
}

static void lcd_command(CoreS3ILI9342CState *s, uint8_t cmd)
{
    if (s->current_cmd == CMD_RAMWR && s->dirty && cmd != CMD_RAMWR) {
        lcd_dump_ppm(s);
    }
    s->current_cmd = cmd;
    s->pending_len = 0;
    s->read_index = 0;
    s->pixel_high_valid = false;
    switch (cmd) {
    case CMD_SWRESET: lcd_reset(s); break;
    case CMD_SLPOUT: s->sleeping = false; break;
    case CMD_DISPON: s->display_on = true; break;
    case CMD_RAMWR:
        s->x = s->x0;
        s->y = s->y0;
        break;
    default: break;
    }
}

static void lcd_data(CoreS3ILI9342CState *s, uint8_t data)
{
    switch (s->current_cmd) {
    case CMD_CASET:
    case CMD_PASET:
        if (s->pending_len < sizeof(s->pending)) {
            s->pending[s->pending_len++] = data;
        }
        if (s->pending_len == 4) {
            uint16_t a = ((uint16_t)s->pending[0] << 8) | s->pending[1];
            uint16_t b = ((uint16_t)s->pending[2] << 8) | s->pending[3];
            if (s->current_cmd == CMD_CASET) {
                s->x0 = a;
                s->x1 = b;
            } else {
                s->y0 = a;
                s->y1 = b;
            }
            s->pending_len = 0;
        }
        break;
    case CMD_MADCTL: s->madctl = data; break;
    case CMD_COLMOD: s->pixel_format = data; break;
    case CMD_RAMWR:
        if (s->pixel_format != 0x55) {
            return;
        }
        if (!s->pixel_high_valid) {
            s->pixel_high = data;
            s->pixel_high_valid = true;
        } else {
            uint16_t raw = ((uint16_t)s->pixel_high << 8) | data;
            s->pixel_high_valid = false;
            lcd_write_pixel565(s, raw);
        }
        break;
    default: break;
    }
}

static uint8_t lcd_read(CoreS3ILI9342CState *s)
{
    if (s->current_cmd == CMD_RDDID) {
        static const uint8_t id[] = {0x00, 0x00, 0x00, 0xe3};
        uint8_t v = id[s->read_index % ARRAY_SIZE(id)];
        s->read_index++;
        return v;
    }
    return 0;
}

static uint32_t lcd_transfer(SSIPeripheral *dev, uint32_t value)
{
    CoreS3ILI9342CState *s = CORES3_ILI9342C(dev);
    uint8_t v = value & 0xff;
    uint8_t ret = lcd_read(s);
    if (s->dc) {
        lcd_data(s, v);
    } else {
        lcd_command(s, v);
    }
    return ret;
}

static int lcd_set_cs(SSIPeripheral *dev, bool level)
{
    return 0;
}

static void lcd_set_dc(void *opaque, int n, int level)
{
    CoreS3ILI9342CState *s = CORES3_ILI9342C(opaque);
    s->dc = !!level;
}

static void lcd_realize(SSIPeripheral *dev, Error **errp)
{
    CoreS3ILI9342CState *s = CORES3_ILI9342C(dev);
    lcd_reset(s);
    qdev_init_gpio_in_named(DEVICE(dev), lcd_set_dc, "dc", 1);
    qemu_log("CoreS3 ILI9342C attached, framebuffer output: %s\n",
             s->output ? s->output : "(disabled)");
}

static Property lcd_properties[] = {
    DEFINE_PROP_STRING("output", CoreS3ILI9342CState, output),
    DEFINE_PROP_END_OF_LIST(),
};

static void lcd_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *ssc = SSI_PERIPHERAL_CLASS(klass);
    ssc->realize = lcd_realize;
    ssc->transfer = lcd_transfer;
    ssc->set_cs = lcd_set_cs;
    ssc->cs_polarity = SSI_CS_LOW;
    device_class_set_props(dc, lcd_properties);
}

static const TypeInfo lcd_info = {
    .name = TYPE_CORES3_ILI9342C,
    .parent = TYPE_SSI_PERIPHERAL,
    .instance_size = sizeof(CoreS3ILI9342CState),
    .class_init = lcd_class_init,
};

static void lcd_register_types(void)
{
    type_register_static(&lcd_info);
}

type_init(lcd_register_types)
