#!/usr/bin/env python3
"""Extend the SPI2 overlay into a real CoreS3 wiring experiment.

This stage keeps the already-tested SPI2/ILI9342C model, then adds:
- digital GPIO OUT/ENABLE/IN behavior,
- GPIO3 -> LCD CS and GPIO35 -> LCD D/C,
- GPIO12/11 bit-banged I2C through QEMU's bitbang bridge,
- ESP32 I2C0 at the real ESP32-S3 MMIO address,
- one shared I2C bus for both bit-bang and hardware-controller traffic,
- small register-backed stand-ins for AXP2101, AW9523B and GC0308.

The goal is deliberately narrow: let the unmodified M5GFX CoreS3 autodetect path
see the same devices it checks on physical hardware, without adding SIM-only
shortcuts to the firmware.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected one source anchor, found {count}")
    return text.replace(old, new, 1)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("qemu_source", type=Path)
    ap.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[1])
    args = ap.parse_args()

    qemu = args.qemu_source.resolve()
    root = args.repo_root.resolve()

    # First apply the already validated SPI2 + ILI9342C overlay.
    subprocess.run(
        [sys.executable, str(root / "tools" / "prepare_cores3_qemu.py"), str(qemu),
         "--repo-root", str(root)],
        check=True,
    )

    overlay = root / "sim" / "cores3" / "qemu-overlay"
    for rel in [
        Path("include/hw/gpio/esp32_gpio.h"),
        Path("hw/gpio/esp32_gpio.c"),
    ]:
        src = overlay / rel
        dst = qemu / rel
        if not src.exists():
            raise RuntimeError(f"missing CoreS3 GPIO overlay: {src}")
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)

    # Enable the generic pieces used by the virtual CoreS3 I2C fabric.
    kconfig = qemu / "hw" / "xtensa" / "Kconfig"
    text = kconfig.read_text()
    old = """config XTENSA_ESP32S3
    bool
    default y
    depends on XTENSA
    select SSI
    select SSI_M25P80
    select UNIMP
    select OPENCORES_ETH
    select DWC_SDMMC
    select TMP105
    select ESP_RGB
"""
    new = old + "    select I2C\n    select BITBANG_I2C\n    select AT24C\n"
    text = replace_once(text, old, new, "ESP32-S3 Kconfig")
    kconfig.write_text(text)

    # Log the two kinds of transactions separately. These markers make CI prove
    # that M5GFX crossed both the GPIO bit-bang probe and the controller-backed
    # register-read phase.
    bitbang = qemu / "hw" / "i2c" / "bitbang_i2c.c"
    text = bitbang.read_text()
    text = replace_once(
        text,
        '#include "qemu/osdep.h"\n',
        '#include "qemu/osdep.h"\n#include "qemu/log.h"\n',
        "bitbang qemu log include",
    )
    old = """            ret = i2c_start_transfer(i2c->bus, i2c->current_addr >> 1,
                                     i2c->current_addr & 1);
"""
    new = old + """            if ((i2c->current_addr >> 1) == 0x34 ||
                (i2c->current_addr >> 1) == 0x58 ||
                (i2c->current_addr >> 1) == 0x21) {
                qemu_log("CORES3_I2C_BITBANG:addr=0x%02x rw=%u %s\\n",
                         i2c->current_addr >> 1, i2c->current_addr & 1,
                         ret ? "NACK" : "ACK");
            }
"""
    text = replace_once(text, old, new, "bitbang address logging")
    bitbang.write_text(text)

    hw_i2c = qemu / "hw" / "i2c" / "esp32_i2c.c"
    text = hw_i2c.read_text()
    old = """                    if (i2c_start_transfer(s->bus, addr, is_read) != 0) {
                        /* NACK */
"""
    new = """                    int start_result = i2c_start_transfer(s->bus, addr, is_read);
                    if (addr == 0x34 || addr == 0x58 || addr == 0x21) {
                        qemu_log("CORES3_I2C_HW:addr=0x%02x rw=%u %s\\n",
                                 addr, is_read, start_result ? "NACK" : "ACK");
                    }
                    if (start_result != 0) {
                        /* NACK */
"""
    text = replace_once(text, old, new, "hardware I2C address logging")
    hw_i2c.write_text(text)

    soc_path = qemu / "hw" / "xtensa" / "esp32s3.c"
    text = soc_path.read_text()

    text = replace_once(
        text,
        '#include "hw/i2c/i2c.h"\n',
        '#include "hw/i2c/i2c.h"\n'
        '#include "hw/i2c/esp32_i2c.h"\n'
        '#include "hw/i2c/bitbang_i2c.h"\n'
        '#include "hw/nvram/eeprom_at24c.h"\n',
        "CoreS3 I2C includes",
    )

    text = replace_once(
        text,
        "    ESP32S3GPIOState gpio;\n",
        "    ESP32S3GPIOState gpio;\n    Esp32I2CState i2c0;\n",
        "ESP32-S3 I2C state",
    )

    text = replace_once(
        text,
        '#define ESP32S3_IO_WARNING  0\n',
        '#define ESP32S3_IO_WARNING  0\n\n'
        'static const uint8_t cores3_axp2101_rom[256] = { [0x03] = 0x4a };\n'
        'static const uint8_t cores3_aw9523_rom[256]  = { [0x10] = 0x23 };\n'
        'static const uint8_t cores3_gc0308_rom[256]  = { [0x00] = 0x9b };\n',
        "CoreS3 virtual register ROMs",
    )

    text = replace_once(
        text,
        '    object_initialize_child(OBJECT(ss), "gpio", &ss->gpio, TYPE_ESP32S3_GPIO);\n',
        '    object_initialize_child(OBJECT(ss), "gpio", &ss->gpio, TYPE_ESP32S3_GPIO);\n'
        '    object_initialize_child(OBJECT(ss), "i2c0", &ss->i2c0, TYPE_ESP32_I2C);\n',
        "ESP32-S3 I2C child",
    )

    # Replace the synthetic SPI-controller CS/DC wiring with the physical CoreS3
    # board wiring. SPI2 now only clocks bytes; GPIO3 and GPIO35 decide how the
    # LCD interprets them.
    old = """        qdev_connect_gpio_out_named(DEVICE(&ss->spi2), SSI_GPIO_CS, 0,
                                    qdev_get_gpio_in_named(lcd, SSI_GPIO_CS, 0));
        qdev_connect_gpio_out_named(DEVICE(&ss->spi2), "dc", 0,
                                    qdev_get_gpio_in_named(lcd, "dc", 0));
"""
    new = """        qdev_connect_gpio_out(DEVICE(&ss->gpio), 3,
                               qdev_get_gpio_in_named(lcd, SSI_GPIO_CS, 0));
        qdev_connect_gpio_out(DEVICE(&ss->gpio), 35,
                               qdev_get_gpio_in_named(lcd, "dc", 0));
"""
    text = replace_once(text, old, new, "CoreS3 physical LCD CS/DC wiring")

    gpio_block = """    /* GPIO realization */
    {
        sysbus_realize(SYS_BUS_DEVICE(&ss->gpio), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->gpio), 0);
        memory_region_add_subregion_overlap(sys_mem, DR_REG_GPIO_BASE, mr, 0);
    }
"""

    cores3_i2c_block = gpio_block + """

    /* CoreS3 internal I2C fabric. GPIO12=SDA and GPIO11=SCL, exactly as on
     * the physical CoreS3. Bit-bang probing and I2C0 share one QEMU bus. */
    {
        DeviceState *gpio_i2c = qdev_new(TYPE_GPIO_I2C);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(gpio_i2c), &error_fatal);
        I2CBus *cores3_bus = I2C_BUS(qdev_get_child_bus(gpio_i2c, "i2c"));

        qdev_connect_gpio_out(DEVICE(&ss->gpio), 12,
                              qdev_get_gpio_in(gpio_i2c, BITBANG_I2C_SDA));
        qdev_connect_gpio_out(DEVICE(&ss->gpio), 11,
                              qdev_get_gpio_in(gpio_i2c, BITBANG_I2C_SCL));
        qdev_connect_gpio_out(gpio_i2c, 0,
                              qdev_get_gpio_in(DEVICE(&ss->gpio), 12));

        sysbus_realize(SYS_BUS_DEVICE(&ss->i2c0), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->i2c0), 0);
        memory_region_add_subregion_overlap(sys_mem, 0x60013000, mr, 0);
        /* ESP32-S3 peripheral interrupt source 42 = I2C_EXT0. */
        sysbus_connect_irq(SYS_BUS_DEVICE(&ss->i2c0), 0,
                           qdev_get_gpio_in(intmatrix_dev, 42));
        ss->i2c0.bus = cores3_bus;

        at24c_eeprom_init_rom(cores3_bus, 0x34, 256,
                              cores3_axp2101_rom, sizeof(cores3_axp2101_rom));
        at24c_eeprom_init_rom(cores3_bus, 0x58, 256,
                              cores3_aw9523_rom, sizeof(cores3_aw9523_rom));
        at24c_eeprom_init_rom(cores3_bus, 0x21, 256,
                              cores3_gc0308_rom, sizeof(cores3_gc0308_rom));
        qemu_log("CoreS3 virtual I2C fabric attached: GPIO12=SDA GPIO11=SCL + I2C0\\n");
    }
"""
    text = replace_once(text, gpio_block, cores3_i2c_block, "CoreS3 I2C fabric")

    soc_path.write_text(text)

    print("CORES3_QEMU_REAL_WIRING_OVERLAY:PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
