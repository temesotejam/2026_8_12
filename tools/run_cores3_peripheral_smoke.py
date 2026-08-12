#!/usr/bin/env python3
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sim.cores3.peripheral_models import AXP2101Model, AW9523BModel, ILI9342CModel


def main() -> int:
    out_dir = ROOT / "artifacts"
    out_dir.mkdir(exist_ok=True)

    axp = AXP2101Model()
    aw = AW9523BModel()
    lcd = ILI9342CModel()

    assert axp.read(0x03) == b"\x4a", "AXP2101 chip ID must match M5Unified expectation"

    # Model the startup actions needed by the future CoreS3 bridge.
    axp.write(0x99, b"\x1c")
    axp.write(0x90, bytes([axp.reg[0x90] | 0x80]))
    assert axp.dldo1_enabled

    aw.set_pin("P1_1_LCD_RST", 0)
    aw.set_pin("P1_1_LCD_RST", 1)
    assert aw.pin_state["P1_1_LCD_RST"] == 1

    # Typical LCD bring-up state followed by SPI-style command/data transactions.
    lcd.command(lcd.SLPOUT)
    lcd.command(lcd.PIXFMT)
    lcd.data(b"\x55")  # RGB565
    lcd.command(lcd.MADCTL)
    lcd.data(b"\x00")
    lcd.command(lcd.DISPON)

    # Produce an unmistakable reference frame through the ILI9342C RAM-write path.
    lcd.fill_rect(0, 0, 320, 240, (8, 12, 20))
    lcd.fill_rect(12, 12, 296, 42, (0, 180, 100))
    lcd.fill_rect(24, 76, 130, 120, (230, 65, 55))
    lcd.fill_rect(170, 76, 126, 120, (45, 110, 240))
    lcd.fill_rect(24, 210, 272, 18, (245, 200, 35))

    # Pixel checks prove that address-window + RAMWR + RGB565 decode worked.
    checks = {
        (0, 0): (8, 12, 20),
        (20, 20): (0, 180, 100),
        (40, 100): (230, 65, 55),
        (200, 100): (45, 110, 240),
        (100, 220): (245, 200, 35),
    }
    for point, expected in checks.items():
        actual = lcd.pixel(*point)
        # RGB565 quantization means decoded values are close, not bit-identical.
        assert all(abs(a - e) <= 8 for a, e in zip(actual, expected)), (point, actual, expected)

    png_path = out_dir / "cores3-screen.png"
    lcd.save_png(str(png_path))

    print(f"AXP2101_ID:0x{axp.read(0x03)[0]:02X}")
    print(f"AXP2101_DLDO1:{axp.dldo1_millivolts}mV enabled={int(axp.dldo1_enabled)}")
    print(f"AW9523B_LCD_RST:{aw.pin_state['P1_1_LCD_RST']}")
    print(f"ILI9342C_STATE:sleep={int(lcd.sleep)} display_on={int(lcd.display_on)} pixfmt=0x{lcd.pixel_format:02X}")
    print(f"SCREEN_ARTIFACT:{png_path.relative_to(ROOT)}")
    print("CORES3_PERIPHERAL_MODEL_CHECK:PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
