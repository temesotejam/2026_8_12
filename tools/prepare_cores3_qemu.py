#!/usr/bin/env python3
"""Apply the CoreS3 SPI2/LCD experimental overlay to Espressif QEMU.

Pinned to Espressif QEMU commit 40edccac415693c5130f91c01d84176ae6008566
(release esp-develop-9.2.2-20260417). The script deliberately fails when
source anchors change, rather than silently applying a partial patch.
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

PINNED_QEMU_COMMIT = "40edccac415693c5130f91c01d84176ae6008566"


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
    overlay = root / "sim" / "cores3" / "qemu-overlay"
    if not (qemu / "hw" / "xtensa" / "esp32s3.c").exists():
        raise RuntimeError(f"not an Espressif QEMU source tree: {qemu}")

    copies = {
        overlay / "include" / "hw" / "ssi" / "esp32s3_gpspi.h": qemu / "include" / "hw" / "ssi" / "esp32s3_gpspi.h",
        overlay / "hw" / "ssi" / "esp32s3_gpspi.c": qemu / "hw" / "ssi" / "esp32s3_gpspi.c",
        overlay / "hw" / "display" / "cores3_ili9342c.c": qemu / "hw" / "display" / "cores3_ili9342c.c",
    }
    for src, dst in copies.items():
        if not src.exists():
            raise RuntimeError(f"missing overlay source: {src}")
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)

    ssi_meson = qemu / "hw" / "ssi" / "meson.build"
    text = ssi_meson.read_text()
    anchor = "system_ss.add(when: 'CONFIG_XTENSA_ESP32S3', if_true: files('esp32s3_spi.c'))\n"
    text = replace_once(text, anchor, anchor + "system_ss.add(when: 'CONFIG_XTENSA_ESP32S3', if_true: files('esp32s3_gpspi.c'))\n", "ssi meson")
    ssi_meson.write_text(text)

    display_meson = qemu / "hw" / "display" / "meson.build"
    text = display_meson.read_text()
    anchor = "system_ss.add(when: 'CONFIG_ESP_RGB', if_true: files('esp_rgb.c'))\n"
    text = replace_once(text, anchor, anchor + "system_ss.add(when: 'CONFIG_XTENSA_ESP32S3', if_true: files('cores3_ili9342c.c'))\n", "display meson")
    display_meson.write_text(text)

    soc_path = qemu / "hw" / "xtensa" / "esp32s3.c"
    text = soc_path.read_text()
    text = replace_once(text,
        '#include "hw/ssi/esp32s3_spi.h"\n',
        '#include "hw/ssi/esp32s3_spi.h"\n#include "hw/ssi/esp32s3_gpspi.h"\n',
        "esp32s3 include")
    text = replace_once(text,
        "    ESP32S3SpiState spi1;\n",
        "    ESP32S3SpiState spi1;\n    ESP32S3GpspiState spi2;\n",
        "esp32s3 state")
    text = replace_once(text,
        '    object_initialize_child(OBJECT(ss), "spi1", &ss->spi1, TYPE_ESP32S3_SPI);\n',
        '    object_initialize_child(OBJECT(ss), "spi1", &ss->spi1, TYPE_ESP32S3_SPI);\n'
        '    object_initialize_child(OBJECT(ss), "spi2", &ss->spi2, TYPE_ESP32S3_GPSPI);\n',
        "esp32s3 child")

    anchor = "    /* (Extmem) Cache realization */\n"
    block = '''    /* Experimental CoreS3 general-purpose SPI2 + ILI9342C. */
    {
        const hwaddr cores3_spi2_base = 0x60024000;
        sysbus_realize(SYS_BUS_DEVICE(&ss->spi2), &error_fatal);
        MemoryRegion *mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&ss->spi2), 0);
        memory_region_add_subregion_overlap(sys_mem, cores3_spi2_base, mr, 0);

        BusState *bus = qdev_get_child_bus(DEVICE(&ss->spi2), "spi");
        DeviceState *lcd = qdev_new("cores3-ili9342c");
        qdev_prop_set_string(lcd, "output", "cores3-screen.ppm");
        qdev_realize_and_unref(lcd, bus, &error_fatal);
        qdev_connect_gpio_out_named(DEVICE(&ss->spi2), SSI_GPIO_CS, 0,
                                    qdev_get_gpio_in_named(lcd, SSI_GPIO_CS, 0));
        qdev_connect_gpio_out_named(DEVICE(&ss->spi2), "dc", 0,
                                    qdev_get_gpio_in_named(lcd, "dc", 0));
    }

'''
    text = replace_once(text, anchor, block + anchor, "esp32s3 SPI2 block")
    soc_path.write_text(text)

    print(f"CORES3_QEMU_OVERLAY:PASS pinned={PINNED_QEMU_COMMIT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
