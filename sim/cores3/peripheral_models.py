from __future__ import annotations

import binascii
import struct
import zlib


class RegisterDevice:
    def __init__(self, address: int, defaults: dict[int, int] | None = None):
        self.address = address
        self.reg = bytearray(256)
        if defaults:
            for key, value in defaults.items():
                self.reg[key & 0xFF] = value & 0xFF

    def read(self, register: int, length: int = 1) -> bytes:
        register &= 0xFF
        return bytes(self.reg[register: register + length])

    def write(self, register: int, data: bytes) -> None:
        register &= 0xFF
        end = min(256, register + len(data))
        self.reg[register:end] = data[: end - register]


class AXP2101Model(RegisterDevice):
    """Subset needed for CoreS3 startup work.

    Register 0x03 returns the AXP2101 chip ID expected by M5Unified.
    DLDO1 uses voltage register 0x99 and enable bit 7 of register 0x90,
    matching M5Unified's AXP2101_Class implementation.
    """

    def __init__(self):
        super().__init__(0x34, {0x03: 0x4A})

    @property
    def dldo1_enabled(self) -> bool:
        return bool(self.reg[0x90] & 0x80)

    @property
    def dldo1_millivolts(self) -> int:
        return 500 + min(self.reg[0x99], 0x1C) * 100


class AW9523BModel(RegisterDevice):
    """Small register/pin model for CoreS3 reset/control lines.

    It intentionally starts as a register-file model. The named pin states
    provide the stable interface that later QEMU-I2C bridging will drive.
    """

    def __init__(self):
        super().__init__(0x58)
        self.pin_state = {
            "P0_0_TOUCH_RST": 1,
            "P0_2_SPK_RST": 1,
            "P1_0_CAM_RST": 1,
            "P1_1_LCD_RST": 1,
            "P1_2_TOUCH_INT": 1,
            "P1_3_SPK_INT": 1,
        }

    def set_pin(self, name: str, value: int) -> None:
        if name not in self.pin_state:
            raise KeyError(name)
        self.pin_state[name] = 1 if value else 0


class ILI9342CModel:
    WIDTH = 320
    HEIGHT = 240

    CASET = 0x2A
    PASET = 0x2B
    RAMWR = 0x2C
    MADCTL = 0x36
    PIXFMT = 0x3A
    SLPOUT = 0x11
    DISPON = 0x29
    SWRESET = 0x01

    def __init__(self):
        self.framebuffer = bytearray(self.WIDTH * self.HEIGHT * 3)
        self.sleep = True
        self.display_on = False
        self.pixel_format = 0x55
        self.madctl = 0
        self.x0 = 0
        self.x1 = self.WIDTH - 1
        self.y0 = 0
        self.y1 = self.HEIGHT - 1
        self._write_x = 0
        self._write_y = 0
        self._ram_write = False
        self._pending_command: int | None = None
        self._pending = bytearray()

    def reset(self) -> None:
        self.__init__()

    def command(self, cmd: int) -> None:
        cmd &= 0xFF
        self._pending_command = cmd
        self._pending.clear()
        self._ram_write = cmd == self.RAMWR
        if cmd == self.SWRESET:
            self.reset()
            return
        if cmd == self.SLPOUT:
            self.sleep = False
        elif cmd == self.DISPON:
            self.display_on = True
        elif cmd == self.RAMWR:
            self._write_x = self.x0
            self._write_y = self.y0

    def data(self, payload: bytes) -> None:
        if self._ram_write:
            self._write_rgb565(payload)
            return
        self._pending.extend(payload)
        cmd = self._pending_command
        if cmd == self.CASET and len(self._pending) >= 4:
            self.x0, self.x1 = struct.unpack(">HH", self._pending[:4])
            self._pending.clear()
        elif cmd == self.PASET and len(self._pending) >= 4:
            self.y0, self.y1 = struct.unpack(">HH", self._pending[:4])
            self._pending.clear()
        elif cmd == self.MADCTL and self._pending:
            self.madctl = self._pending[0]
            self._pending.clear()
        elif cmd == self.PIXFMT and self._pending:
            self.pixel_format = self._pending[0]
            self._pending.clear()

    def _write_rgb565(self, payload: bytes) -> None:
        if len(payload) % 2:
            raise ValueError("RGB565 payload must contain complete pixels")
        for i in range(0, len(payload), 2):
            raw = (payload[i] << 8) | payload[i + 1]
            r5 = (raw >> 11) & 0x1F
            g6 = (raw >> 5) & 0x3F
            b5 = raw & 0x1F
            rgb = (
                (r5 * 255 + 15) // 31,
                (g6 * 255 + 31) // 63,
                (b5 * 255 + 15) // 31,
            )
            if 0 <= self._write_x < self.WIDTH and 0 <= self._write_y < self.HEIGHT:
                pos = (self._write_y * self.WIDTH + self._write_x) * 3
                self.framebuffer[pos:pos + 3] = bytes(rgb)
            self._write_x += 1
            if self._write_x > self.x1:
                self._write_x = self.x0
                self._write_y += 1
                if self._write_y > self.y1:
                    self._write_y = self.y0

    def set_window(self, x: int, y: int, width: int, height: int) -> None:
        self.command(self.CASET)
        self.data(struct.pack(">HH", x, x + width - 1))
        self.command(self.PASET)
        self.data(struct.pack(">HH", y, y + height - 1))
        self.command(self.RAMWR)

    @staticmethod
    def rgb565(r: int, g: int, b: int) -> bytes:
        raw = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
        return struct.pack(">H", raw)

    def fill_rect(self, x: int, y: int, width: int, height: int, rgb: tuple[int, int, int]) -> None:
        self.set_window(x, y, width, height)
        px = self.rgb565(*rgb)
        self.data(px * (width * height))

    def pixel(self, x: int, y: int) -> tuple[int, int, int]:
        pos = (y * self.WIDTH + x) * 3
        return tuple(self.framebuffer[pos:pos + 3])

    def save_png(self, path: str) -> None:
        # PNG writer kept dependency-free so it runs on bare GitHub runners.
        def chunk(kind: bytes, data: bytes) -> bytes:
            return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", binascii.crc32(kind + data) & 0xFFFFFFFF)

        scan = bytearray()
        row_bytes = self.WIDTH * 3
        for y in range(self.HEIGHT):
            scan.append(0)
            start = y * row_bytes
            scan.extend(self.framebuffer[start:start + row_bytes])
        png = bytearray(b"\x89PNG\r\n\x1a\n")
        png.extend(chunk(b"IHDR", struct.pack(">IIBBBBB", self.WIDTH, self.HEIGHT, 8, 2, 0, 0, 0)))
        png.extend(chunk(b"IDAT", zlib.compress(bytes(scan), 9)))
        png.extend(chunk(b"IEND", b""))
        with open(path, "wb") as f:
            f.write(png)
