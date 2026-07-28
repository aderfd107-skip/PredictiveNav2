"""Unit tests for the A* planner and grid graph."""

import math
import os
import sys
import tempfile
from pathlib import Path

import pytest

# Make inspection_core importable from the test directory
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from inspection_core.grid_graph import GridGraph, OCC_FREE, OCC_OCCUPIED, Cell
from inspection_core.astar import (
    astar,
    cell_path_length,
    cell_path_to_world_path,
    world_path_length,
)


# ---------------------------------------------------------------------------
# helpers — build tiny synthetic PGM + YAML for deterministic testing
# ---------------------------------------------------------------------------

def _write_pgm(path: str, pixels: list[list[int]], max_val: int = 255) -> None:
    """Write a binary P5 PGM from a 2-D list of 0..max_val values."""
    height = len(pixels)
    width = len(pixels[0]) if height else 0
    with open(path, "wb") as fh:
        fh.write(f"P5\n{width} {height}\n{max_val}\n".encode())
        for row in pixels:
            for val in row:
                fh.write(bytes([val]))


def _make_map(tmpdir: str, pixels: list[list[int]], **meta) -> str:
    """Create a .pgm + .yaml pair, return the yaml path."""
    pgm = os.path.join(tmpdir, "test.pgm")
    yaml = os.path.join(tmpdir, "test.yaml")
    _write_pgm(pgm, pixels)

    defaults = {
        "image": "test.pgm",
        "mode": "trinary",
        "resolution": 0.05,
        "origin": [0.0, 0.0, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    defaults.update(meta)

    import yaml as _yaml
    with open(yaml, "w") as fh:
        _yaml.dump(defaults, fh)
    return yaml


# Pixel conventions matching Nav2 map_saver "trinary" output:
#   254 (white) = free     0 (black) = occupied     205 (gray) = unknown
PIX_FREE = 254
PIX_OCCUPIED = 0


# A 10×10 open grid (all free)
def _open_10x10(tmpdir):
    pixels = [[PIX_FREE] * 10 for _ in range(10)]
    return _make_map(tmpdir, pixels)


# A 10×10 grid with a vertical wall in the middle
def _wall_10x10(tmpdir):
    pixels = [[PIX_FREE] * 10 for _ in range(10)]
    for r in range(10):
        pixels[r][5] = PIX_OCCUPIED  # fully occupied
    return _make_map(tmpdir, pixels)


# ---------------------------------------------------------------------------
# GridGraph
# ---------------------------------------------------------------------------


class TestGridGraph:
    def test_open_grid_basics(self):
        with tempfile.TemporaryDirectory() as td:
            yaml_path = _open_10x10(td)
            g = GridGraph(yaml_path)

            assert g.width == 10
            assert g.height == 10
            assert g.resolution == 0.05
            assert g.is_free(0, 0)
            assert g.is_free(9, 9)

    def test_world_cell_roundtrip(self):
        with tempfile.TemporaryDirectory() as td:
            yaml_path = _open_10x10(td)
            g = GridGraph(yaml_path)
            c = g.world_to_cell(0.25, 0.25)
            assert c == (5, 5)
            wx, wy = g.cell_to_world(5, 5)
            assert abs(wx - 0.275) < 1e-6  # centre of cell 5
            assert abs(wy - 0.275) < 1e-6

    def test_neighbours_mid_grid(self):
        with tempfile.TemporaryDirectory() as td:
            yaml_path = _open_10x10(td)
            g = GridGraph(yaml_path)
            nbs = list(g.neighbours(5, 5))
            assert len(nbs) == 4
            assert (5, 6) in nbs  # up
            assert (6, 5) in nbs  # right
            assert (5, 4) in nbs  # down
            assert (4, 5) in nbs  # left

    def test_neighbours_corner(self):
        with tempfile.TemporaryDirectory() as td:
            yaml_path = _open_10x10(td)
            g = GridGraph(yaml_path)
            nbs = list(g.neighbours(0, 0))
            assert len(nbs) == 2  # only up and right

    def test_neighbours_skip_wall(self):
        with tempfile.TemporaryDirectory() as td:
            yaml_path = _wall_10x10(td)
            g = GridGraph(yaml_path)
            nbs = list(g.neighbours(4, 5))  # cell left of wall
            # right neighbour (5,5) is a wall — should be excluded
            assert (5, 5) not in nbs

    def test_inflation(self):
        with tempfile.TemporaryDirectory() as td:
            yaml_path = _open_10x10(td)
            # Place one occupied cell
            pixels = [[PIX_FREE] * 10 for _ in range(10)]
            pixels[5][5] = PIX_OCCUPIED
            inflated_yaml = _make_map(td, pixels)
            g = GridGraph(inflated_yaml, inflation_radius_m=0.15)  # 3 cells

            assert not g.is_free(5, 5)  # original obstacle
            assert not g.is_free(4, 5)  # within 3 cells
            assert not g.is_free(5, 4)
            assert not g.is_free(3, 5)  # within 3 cells
            assert g.is_free(2, 5)  # outside inflation
            assert g.is_free(8, 8)


# ---------------------------------------------------------------------------
# A*
# ---------------------------------------------------------------------------


class TestAstar:
    def test_straight_line(self):
        with tempfile.TemporaryDirectory() as td:
            g = GridGraph(_open_10x10(td))
            path = astar(g, (0, 0), (9, 0))
            assert path is not None
            assert path[0] == (0, 0)
            assert path[-1] == (9, 0)
            assert len(path) == 10  # 0…9 = 10 cells

    def test_impassable_wall(self):
        with tempfile.TemporaryDirectory() as td:
            g = GridGraph(_wall_10x10(td))
            # Start left of wall, goal right of wall — disconnected
            path = astar(g, (0, 0), (9, 0))
            assert path is None

    def test_start_is_goal(self):
        with tempfile.TemporaryDirectory() as td:
            g = GridGraph(_open_10x10(td))
            path = astar(g, (3, 3), (3, 3))
            assert path == [(3, 3)]

    def test_start_occupied(self):
        with tempfile.TemporaryDirectory() as td:
            g = GridGraph(_wall_10x10(td))
            path = astar(g, (5, 5), (0, 0))  # start in wall
            assert path is None

    def test_goal_occupied(self):
        with tempfile.TemporaryDirectory() as td:
            g = GridGraph(_wall_10x10(td))
            path = astar(g, (0, 0), (5, 5))  # goal in wall
            assert path is None

    def test_out_of_bounds(self):
        with tempfile.TemporaryDirectory() as td:
            g = GridGraph(_open_10x10(td))
            path = astar(g, (-1, 0), (5, 5))
            assert path is None

    def test_path_is_optimal(self):
        with tempfile.TemporaryDirectory() as td:
            g = GridGraph(_open_10x10(td))
            path = astar(g, (0, 0), (3, 4))
            assert path is not None
            # Manhattan distance = 3 + 4 = 7 edges → 8 cells
            assert len(path) == 8

    def test_detour_around_wall(self):
        with tempfile.TemporaryDirectory() as td:
            # Create a wall that forces a detour
            pixels = [[PIX_FREE] * 10 for _ in range(10)]
            for r in range(2, 8):
                pixels[r][5] = PIX_OCCUPIED
            y = _make_map(td, pixels)
            g = GridGraph(y)
            path = astar(g, (4, 5), (6, 5))
            assert path is not None
            # Must go around the wall — length > 2
            assert len(path) > 2


# ---------------------------------------------------------------------------
# path helpers
# ---------------------------------------------------------------------------


class TestPathHelpers:
    def test_cell_path_length(self):
        path = [(0, 0), (1, 0), (2, 0)]
        assert cell_path_length(path) == 2.0

    def test_world_path_length(self):
        with tempfile.TemporaryDirectory() as td:
            g = GridGraph(_open_10x10(td))
            path = [(0, 0), (1, 0), (2, 0)]
            d = world_path_length(g, path)
            assert math.isclose(d, 0.10)  # 2 edges × 0.05 m

    def test_cell_to_world_path_step(self):
        with tempfile.TemporaryDirectory() as td:
            g = GridGraph(_open_10x10(td))
            path = [(i, 0) for i in range(10)]  # 10 cells
            wps = cell_path_to_world_path(g, path, step=3)
            # i=0 (first), i=3, i=6, i=9 (last) = 4 waypoints
            assert len(wps) == 4
