#!/usr/bin/env python3

import argparse
from pathlib import Path


def parse_size(text: str) -> int:
    value = text.strip().lower()
    multiplier = 1
    if value.endswith("mb"):
        multiplier = 1024 * 1024
        value = value[:-2]
    elif value.endswith("kb"):
        multiplier = 1024
        value = value[:-2]
    return int(value, 0) * multiplier


def parse_segment(text: str) -> tuple[int, Path]:
    try:
        offset_text, path_text = text.split(":", 1)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("segment must be OFFSET:PATH") from exc
    return int(offset_text, 0), Path(path_text)


def main() -> int:
    parser = argparse.ArgumentParser(description="Build an ESP flash image from binary segments.")
    parser.add_argument("--size", required=True, type=parse_size)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--segment", action="append", required=True, type=parse_segment)
    args = parser.parse_args()

    image = bytearray(b"\xff") * args.size
    occupied: list[tuple[int, int, Path]] = []

    for offset, path in sorted(args.segment, key=lambda item: item[0]):
        data = path.read_bytes()
        end = offset + len(data)
        if offset < 0 or end > args.size:
            raise SystemExit(f"segment {path} does not fit: 0x{offset:x}-0x{end:x}")
        for used_start, used_end, used_path in occupied:
            if offset < used_end and end > used_start:
                raise SystemExit(f"segment overlap: {path} overlaps {used_path}")
        image[offset:end] = data
        occupied.append((offset, end, path))
        print(f"FLASH_SEGMENT offset=0x{offset:x} size={len(data)} path={path}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(image)
    print(f"FLASH_IMAGE size={len(image)} output={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
