#!/usr/bin/env python3
from __future__ import annotations

import argparse
import binascii
import struct
import zlib
from pathlib import Path


def png_chunk(kind: bytes, payload: bytes) -> bytes:
    return (struct.pack(">I", len(payload)) + kind + payload +
            struct.pack(">I", binascii.crc32(kind + payload) & 0xffffffff))


def read_ppm(path: Path) -> tuple[int, int, bytes]:
    data = path.read_bytes()
    if not data.startswith(b"P6\n"):
        raise ValueError("expected binary P6 PPM")
    pos = 3
    tokens: list[bytes] = []
    while len(tokens) < 3:
        while pos < len(data) and chr(data[pos]).isspace():
            pos += 1
        if pos < len(data) and data[pos] == ord('#'):
            while pos < len(data) and data[pos] != 10:
                pos += 1
            continue
        start = pos
        while pos < len(data) and not chr(data[pos]).isspace():
            pos += 1
        tokens.append(data[start:pos])
    width, height, maxv = map(int, tokens)
    if maxv != 255:
        raise ValueError("only maxval 255 supported")
    while pos < len(data) and chr(data[pos]).isspace():
        pos += 1
    pixels = data[pos:]
    if len(pixels) != width * height * 3:
        raise ValueError(f"bad pixel length {len(pixels)} for {width}x{height}")
    return width, height, pixels


def write_png(path: Path, width: int, height: int, pixels: bytes) -> None:
    rows = bytearray()
    stride = width * 3
    for y in range(height):
        rows.append(0)
        rows.extend(pixels[y * stride:(y + 1) * stride])
    out = bytearray(b"\x89PNG\r\n\x1a\n")
    out += png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
    out += png_chunk(b"IDAT", zlib.compress(bytes(rows), 9))
    out += png_chunk(b"IEND", b"")
    path.write_bytes(out)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("input", type=Path)
    ap.add_argument("output", type=Path)
    args = ap.parse_args()
    width, height, pixels = read_ppm(args.input)
    write_png(args.output, width, height, pixels)
    print(f"PPM_TO_PNG:PASS {width}x{height} bytes={args.output.stat().st_size}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
