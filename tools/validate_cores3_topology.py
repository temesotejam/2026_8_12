#!/usr/bin/env python3
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAP_PATH = ROOT / "sim" / "cores3" / "board_topology.json"


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    data = json.loads(MAP_PATH.read_text(encoding="utf-8"))

    require(data["board"]["name"] == "M5Stack CoreS3", "unexpected board name")
    require(data["board"]["soc"] == "ESP32-S3", "unexpected SoC")

    mbus = {entry["pin"]: entry["signal"] for entry in data["m5_bus"]}
    require(len(mbus) == 30, "M5-Bus must contain exactly 30 pins")
    require(set(mbus) == set(range(1, 31)), "M5-Bus pin numbers must be 1..30")

    ports = data["external_ports"]
    expected_ports = {
        "A": {"yellow_gpio": 2, "yellow_bus_pin": 19, "white_gpio": 1, "white_bus_pin": 20},
        "B": {"yellow_gpio": 9, "yellow_bus_pin": 10, "white_gpio": 8, "white_bus_pin": 4},
        "C": {"yellow_gpio": 17, "yellow_bus_pin": 16, "white_gpio": 18, "white_bus_pin": 15},
    }
    for name, expected in expected_ports.items():
        require(ports[name]["m5_bus"] == expected, f"Port {name} mapping mismatch")
        require(f"GPIO{expected['yellow_gpio']}" in mbus[expected["yellow_bus_pin"]], f"Port {name} yellow/M5-Bus mismatch")
        require(f"GPIO{expected['white_gpio']}" in mbus[expected["white_bus_pin"]], f"Port {name} white/M5-Bus mismatch")
        require(ports[name]["pins"]["black"] == "GND", f"Port {name} black pin must be GND")
        require(ports[name]["pins"]["red"] == "5V", f"Port {name} red pin must be 5V")

    require(ports["A"]["physical_location"] == "CoreS3 main unit", "Port A physical location mismatch")
    require("Base DIN" in ports["B"]["physical_location"], "Port B should be exposed through Base DIN")
    require("Base DIN" in ports["C"]["physical_location"], "Port C should be exposed through Base DIN")

    spi = data["buses"]["display_sd_spi"]
    require(spi["mosi_gpio"] == 37, "shared SPI MOSI must be GPIO37")
    require(spi["sck_gpio"] == 36, "shared SPI SCK must be GPIO36")
    require(set(spi["shared_gpio35_roles"]) == {"LCD_DC", "SD_MISO"}, "GPIO35 shared roles mismatch")
    require(spi["devices"]["ILI9342C"]["cs_gpio"] == 3, "LCD CS mismatch")
    require(spi["devices"]["ILI9342C"]["dc_gpio"] == 35, "LCD DC mismatch")
    require(spi["devices"]["microSD"]["miso_gpio"] == 35, "SD MISO mismatch")
    require(spi["devices"]["microSD"]["cs_gpio"] == 4, "SD CS mismatch")

    i2c = data["buses"]["internal_i2c"]
    require(i2c["sda_gpio"] == 12 and i2c["scl_gpio"] == 11, "internal I2C pins mismatch")
    addresses = {device["name"]: device["address"] for device in i2c["devices"]}
    expected_addresses = {
        "AXP2101": "0x34", "AW9523B": "0x58", "BM8563": "0x51", "BMI270": "0x69",
        "ES7210": "0x40", "AW88298": "0x36", "FT6336U": "0x38", "GC0308": "0x21",
        "LTR-553ALS-WA": "0x23",
    }
    require(addresses == expected_addresses, "internal I2C device table mismatch")
    require(len(set(addresses.values())) == len(addresses), "duplicate internal I2C address")

    aux_devices = data["buses"]["bmi270_aux_i2c"]["devices"]
    require(len(aux_devices) == 1, "unexpected BMI270 auxiliary bus device count")
    require(aux_devices[0]["name"] == "BMM150" and aux_devices[0]["address"] == "0x10", "BMM150 auxiliary mapping mismatch")
    require("BMM150" not in addresses, "BMM150 must not be modeled as directly attached to ESP32-S3 internal I2C")

    camera = data["peripherals"]["camera"]
    require(camera["data_gpio"] == [39, 40, 41, 42, 15, 16, 48, 47], "camera D0..D7 mapping mismatch")
    require((camera["pclk_gpio"], camera["vsync_gpio"], camera["href_gpio"]) == (45, 46, 38), "camera timing pin mismatch")

    audio = data["peripherals"]["audio"]
    require((audio["i2s_bck_gpio"], audio["i2s_wck_gpio"], audio["speaker_dout_gpio"], audio["microphone_din_gpio"], audio["mclk_gpio"]) == (34, 33, 13, 14, 0), "audio/I2S mapping mismatch")

    print(f"CORES3_TOPOLOGY:ports={len(ports)} mbus_pins={len(mbus)} internal_i2c_devices={len(addresses)}")
    print("CORES3_TOPOLOGY_CHECK:PASS")


if __name__ == "__main__":
    main()
