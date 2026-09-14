#!/usr/bin/env python3
"""Generate a static occupancy map directly from this lab's SDF walls.

This is an authoring utility, not a runtime navigation node.  It deliberately
uses the ``static_experiment_geometry`` collision boxes as the source of
truth, so the generated map cannot inherit SLAM rasterisation or loop-closure
error.  A cell is occupied when its area intersects a collision box; this is
conservative by at most one half-cell at each wall face.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET


WORLD_MIN_X = -10.0
WORLD_MIN_Y = -8.0
WORLD_MAX_X = 10.0
WORLD_MAX_Y = 8.0
FREE = 254
OCCUPIED = 0


def parse_wall_boxes(world_path: Path) -> list[tuple[float, float, float, float]]:
    """Return x/y bounds for the static wall collision boxes in the SDF."""
    root = ET.parse(world_path).getroot()
    model = root.find(".//world/model[@name='static_experiment_geometry']")
    if model is None:
        raise ValueError("static_experiment_geometry model was not found")

    boxes = []
    for collision in model.findall(".//collision"):
        size_text = collision.findtext("geometry/box/size")
        pose_text = collision.findtext("pose")
        if size_text is None or pose_text is None:
            continue
        size = [float(value) for value in size_text.split()]
        pose = [float(value) for value in pose_text.split()]
        if len(size) != 3 or len(pose) != 6:
            raise ValueError(f"Malformed box collision {collision.get('name')!r}")
        x, y = pose[:2]
        width, height = size[:2]
        boxes.append((x - width / 2.0, x + width / 2.0,
                      y - height / 2.0, y + height / 2.0))

    if not boxes:
        raise ValueError("No static wall collision boxes were found")
    return boxes


def cell_range(low: float, high: float, origin: float, resolution: float,
               count: int) -> range:
    """Indices of grid cells that have non-zero overlap with [low, high]."""
    first = max(0, math.floor((low - origin) / resolution))
    last = min(count - 1, math.ceil((high - origin) / resolution) - 1)
    return range(first, last + 1)


def generate(world_path: Path, output_path: Path, resolution: float) -> tuple[int, int]:
    width = round((WORLD_MAX_X - WORLD_MIN_X) / resolution)
    height = round((WORLD_MAX_Y - WORLD_MIN_Y) / resolution)
    if not math.isclose(width * resolution, WORLD_MAX_X - WORLD_MIN_X) or not math.isclose(
        height * resolution, WORLD_MAX_Y - WORLD_MIN_Y
    ):
        raise ValueError("Resolution must divide the 20 m x 16 m world exactly")

    image = bytearray([FREE]) * (width * height)
    for min_x, max_x, min_y, max_y in parse_wall_boxes(world_path):
        for map_y in cell_range(min_y, max_y, WORLD_MIN_Y, resolution, height):
            # PGM stores its first row at maximum y; map coordinates start at
            # minimum y, hence the vertical flip here.
            row = height - 1 - map_y
            row_start = row * width
            for map_x in cell_range(min_x, max_x, WORLD_MIN_X, resolution, width):
                image[row_start + map_x] = OCCUPIED

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(
        f"P5\n{width} {height}\n255\n".encode("ascii") + bytes(image)
    )
    return width, height


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--resolution", type=float, default=0.025)
    args = parser.parse_args()

    width, height = generate(args.world, args.output, args.resolution)
    print(f"Wrote {args.output} ({width} x {height}, {args.resolution:.3f} m/cell)")


if __name__ == "__main__":
    main()
