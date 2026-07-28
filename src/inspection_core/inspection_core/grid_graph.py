"""2D occupancy grid graph for path planning.

Reads a ``.pgm`` + ``.yaml`` map pair (the Nav2 / map_server format) and
wraps it as a graph that A* and other planners can search over.

The grid uses **4-connectivity** (up / down / left / right) by default
because it keeps edge-conflict detection simpler when multi-robot planning
is added later.

Coordinate convention (matching ROS / Nav2)::

    world  = (x, y)       in metres
    cell   = (col, row)   zero-based integer indices
"""

from __future__ import annotations

import math
import struct
from pathlib import Path
from typing import Iterator, List, Tuple

import yaml

# ---------------------------------------------------------------------------
# types
# ---------------------------------------------------------------------------

Cell = Tuple[int, int]  # (column, row) — x then y to match world coords
WorldPoint = Tuple[float, float]  # (x, y) in metres

# PGM magic numbers
PGM_BINARY = b"P5"

# Occupancy values (Nav2 convention: 0 = free, 100 = occupied, -1 = unknown)
OCC_FREE = 0
OCC_OCCUPIED = 100
OCC_UNKNOWN = -1


# ---------------------------------------------------------------------------
# lightweight YAML loader (so the module is usable without ROS / ament)
# ---------------------------------------------------------------------------

def _load_yaml(path: str) -> dict:
    with open(path, "r") as fh:
        return yaml.safe_load(fh)


# ---------------------------------------------------------------------------
# GridGraph
# ---------------------------------------------------------------------------


class GridGraph:
    """A 2-D occupancy grid built from a Nav2-format map pair.

    Parameters
    ----------
    yaml_path:
        Path to the ``.yaml`` metadata file.  The image is expected next to it.
    inflation_radius_m:
        Obstacles are inflated by this radius so the planner treats the
        robot as a point while leaving safe clearance.  Set to 0 for an
        uninflated grid.
    """

    def __init__(self, yaml_path: str, inflation_radius_m: float = 0.0) -> None:
        meta = _load_yaml(yaml_path)

        self._image_path = str(Path(yaml_path).parent / meta["image"])
        self._resolution: float = float(meta["resolution"])
        self._origin: Tuple[float, float, float] = (
            float(meta["origin"][0]),
            float(meta["origin"][1]),
            float(meta["origin"][2]),
        )
        self._occupied_thresh: float = float(meta.get("occupied_thresh", 0.65))
        self._free_thresh: float = float(meta.get("free_thresh", 0.196))
        self._negate: bool = bool(meta.get("negate", 0))

        self._raw = self._read_pgm(self._image_path)
        self._height = len(self._raw)  # rows
        self._width = len(self._raw[0]) if self._height else 0  # cols

        self._inflation = inflation_radius_m
        if inflation_radius_m > 0.0:
            self._data = self._inflate(int(inflation_radius_m / self._resolution))
        else:
            self._data = [row[:] for row in self._raw]

    # -- public read-only properties ----------------------------------------

    @property
    def width(self) -> int:
        """Number of columns (x direction)."""
        return self._width

    @property
    def height(self) -> int:
        """Number of rows (y direction)."""
        return self._height

    @property
    def resolution(self) -> float:
        """Metres per cell."""
        return self._resolution

    @property
    def origin(self) -> Tuple[float, float, float]:
        """world (x, y, yaw) of the **bottom-left corner** of cell (0, 0)."""
        return self._origin

    # -- coordinate transforms -----------------------------------------------

    def world_to_cell(self, x: float, y: float) -> Cell:
        """Convert world coordinates *(x, y)* to the nearest cell index."""
        col = int((x - self._origin[0]) / self._resolution)
        row = int((y - self._origin[1]) / self._resolution)
        return (col, row)

    def world_to_cell_checked(self, x: float, y: float) -> Cell | None:
        """Like :meth:`world_to_cell` but returns ``None`` when out of bounds."""
        col, row = self.world_to_cell(x, y)
        if 0 <= col < self._width and 0 <= row < self._height:
            return (col, row)
        return None

    def cell_to_world(self, col: int, row: int) -> WorldPoint:
        """Return the world *(x, y)* of the **centre** of *cell*."""
        x = self._origin[0] + (col + 0.5) * self._resolution
        y = self._origin[1] + (row + 0.5) * self._resolution
        return (x, y)

    def cell_to_world_pose(self, col: int, row: int, yaw: float = 0.0) -> Tuple[float, float, float]:
        """World *(x, y, yaw)* of the centre of *cell*."""
        x, y = self.cell_to_world(col, row)
        return (x, y, yaw)

    # -- occupancy queries ---------------------------------------------------

    def is_free(self, col: int, row: int) -> bool:
        """Return True if the cell is safe to traverse."""
        if not (0 <= col < self._width and 0 <= row < self._height):
            return False
        return self._data[row][col] == OCC_FREE

    def is_occupied(self, col: int, row: int) -> bool:
        """Return True if the cell is an obstacle."""
        if not (0 <= col < self._width and 0 <= row < self._height):
            return True  # out-of-bounds is treated as occupied
        return self._data[row][col] != OCC_FREE

    def occupancy(self, col: int, row: int) -> int:
        """Raw occupancy value (0, 100, or -1)."""
        if not (0 <= col < self._width and 0 <= row < self._height):
            return OCC_OCCUPIED
        return self._data[row][col]

    # -- neighbour helpers ---------------------------------------------------

    _NEIGHBOUR_OFFSETS: List[Tuple[int, int]] = [
        (0, 1),   # up
        (1, 0),   # right
        (0, -1),  # down
        (-1, 0),  # left
    ]

    def neighbours(self, col: int, row: int) -> Iterator[Cell]:
        """Yield **free** 4-connected neighbour cells."""
        for dc, dr in self._NEIGHBOUR_OFFSETS:
            nc, nr = col + dc, row + dr
            if self.is_free(nc, nr):
                yield (nc, nr)

    # -- snap to free -------------------------------------------------------

    def snap_to_free(self, col: int, row: int, max_radius: int = 30) -> Cell | None:
        """Return the nearest free cell within *max_radius* cells (BFS).

        Useful when a goal coordinate falls on an inflated obstacle —
        the planner can use the snapped cell instead of failing outright.
        """
        if self.is_free(col, row):
            return (col, row)

        from collections import deque

        seen = {(col, row)}
        queue: deque[Tuple[int, int, int]] = deque()
        queue.append((col, row, 0))

        while queue:
            c, r, dist = queue.popleft()
            if dist >= max_radius:
                break
            for dc, dr in self._NEIGHBOUR_OFFSETS:
                nc, nr = c + dc, r + dr
                if (nc, nr) in seen:
                    continue
                seen.add((nc, nr))
                if self.is_free(nc, nr):
                    return (nc, nr)
                if dist + 1 < max_radius:
                    queue.append((nc, nr, dist + 1))

        return None

    # -- misc ----------------------------------------------------------------

    def cost(self, _from: Cell, _to: Cell) -> float:
        """Edge cost between adjacent free cells (always 1.0 for a uniform grid)."""
        return 1.0

    def heuristic(self, a: Cell, b: Cell) -> float:
        """Manhattan distance — admissible for 4-connected grids."""
        return float(abs(a[0] - b[0]) + abs(a[1] - b[1]))

    # -- internal ------------------------------------------------------------

    def _read_pgm(self, path: str) -> List[List[int]]:
        """Parse a P5 (binary) PGM and return a 2-D list of occupancy values."""
        with open(path, "rb") as fh:
            magic = fh.readline().strip()
            if magic != PGM_BINARY:
                raise ValueError(f"Only P5 PGM is supported, got {magic!r}")

            # Skip comment lines
            line = fh.readline()
            while line.startswith(b"#"):
                line = fh.readline()

            cols, rows = map(int, line.split())
            max_val = int(fh.readline().strip())

            raw = fh.read()
            if max_val <= 255:
                pixels = list(raw)
            else:
                pixels = list(struct.unpack(f">{len(raw)//2}H", raw))

        grid: List[List[int]] = []
        idx = 0
        for r in range(rows):
            grid.append([])
            for _c in range(cols):
                val = pixels[idx]
                idx += 1
                if self._negate:
                    val = max_val - val

                # Normalise to Nav2 convention:
                #   dark pixels  → high occupancy probability → OCCUPIED
                #   bright pixels → low occupancy probability  → FREE
                #   in between   → UNKNOWN
                pct = val / max_val if max_val > 0 else 0.0
                if pct <= self._free_thresh:
                    occ = OCC_OCCUPIED
                elif pct >= self._occupied_thresh:
                    occ = OCC_FREE
                else:
                    occ = OCC_UNKNOWN
                grid[-1].append(occ)

        return grid

    def _inflate(self, radius_cells: int) -> List[List[int]]:
        """Dilate obstacles by *radius_cells* (Chebyshev distance)."""
        if radius_cells <= 0:
            return [row[:] for row in self._raw]

        h, w = self._height, self._width
        inflated = [[OCC_FREE] * w for _ in range(h)]

        for r in range(h):
            for c in range(w):
                if self._raw[r][c] != OCC_FREE:
                    inflated[r][c] = self._raw[r][c]

        for r in range(h):
            for c in range(w):
                if self._raw[r][c] != OCC_FREE:
                    r_min = max(0, r - radius_cells)
                    r_max = min(h - 1, r + radius_cells)
                    c_min = max(0, c - radius_cells)
                    c_max = min(w - 1, c + radius_cells)
                    for rr in range(r_min, r_max + 1):
                        for cc in range(c_min, c_max + 1):
                            inflated[rr][cc] = OCC_OCCUPIED

        return inflated
