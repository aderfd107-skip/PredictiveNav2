#!/usr/bin/env python3
"""Offline A* demo — loads the real map, plans a path, and plots it.

Usage::

    python3 -m inspection_core.offline_demo
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path

from .astar import astar, cell_path_length, world_path_length
from .grid_graph import GridGraph


def _default_map() -> str:
    """Best-guess path to the office map inside this workspace."""
    # When running from the repo root …
    candidate = os.path.join(
        os.path.dirname(__file__), "..", "..", "..",
        "src", "mrt_simulation", "maps", "office_service_mvp.yaml",
    )
    if os.path.exists(candidate):
        return candidate
    # Fallback for an editable install
    return "src/mrt_simulation/maps/office_service_mvp.yaml"


def main() -> None:
    parser = argparse.ArgumentParser(description="Offline A* path planner demo")
    parser.add_argument(
        "--map", default=_default_map(),
        help="Path to the map .yaml file",
    )
    parser.add_argument(
        "--inflation", type=float, default=0.3,
        help="Obstacle inflation radius in metres (default 0.3 m)",
    )
    parser.add_argument(
        "--start", nargs=2, type=float, metavar=("X", "Y"),
        default=[2.0, -1.5],
        help="Start world position (default for charging zone)",
    )
    parser.add_argument(
        "--goal", nargs=2, type=float, metavar=("X", "Y"),
        default=[-4.7, 1.6],
        help="Goal world position (default for Meeting A)",
    )
    args = parser.parse_args()

    print(f"Loading map: {args.map}")
    graph = GridGraph(args.map, inflation_radius_m=args.inflation)
    print(
        f"  size  : {graph.width} × {graph.height} cells"
        f"  ({graph.width * graph.resolution:.1f} ×"
        f" {graph.height * graph.resolution:.1f} m)"
    )
    print(f"  origin: ({graph.origin[0]:.3f}, {graph.origin[1]:.3f})")
    print(f"  resolution: {graph.resolution:.3f} m/cell")
    print(f"  inflation: {args.inflation:.2f} m")

    start_cell = graph.world_to_cell(*args.start)
    goal_cell = graph.world_to_cell(*args.goal)

    print(f"\nStart: world {tuple(args.start)} → cell {start_cell}")
    print(f"Goal : world {tuple(args.goal)} → cell {goal_cell}")

    if not graph.is_free(*start_cell):
        print("[FAIL] Start cell is not free (inside obstacle or out of bounds).")
        return
    if not graph.is_free(*goal_cell):
        print("[FAIL] Goal cell is not free (inside obstacle or out of bounds).")
        return

    print("\nRunning A* …")
    path = astar(graph, start_cell, goal_cell)

    if path is None:
        print("[FAIL] No path found — start and goal are disconnected.")
        return

    cells = cell_path_length(path)
    dist = world_path_length(graph, path)
    print(f"[OK] Path found: {len(path)} cells, {cells} edges, {dist:.2f} m")

    # Optional visualization
    try:
        _plot(graph, path, start_cell, goal_cell, args)
    except ImportError:
        print("\n(matplotlib not available — skipping plot)")


# ---------------------------------------------------------------------------
# matplotlib visualisation
# ---------------------------------------------------------------------------


def _plot(graph, path, start, goal, args) -> None:
    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    import numpy as np

    # Build a 2-D image (0 = free, 1 = occupied/unknown)
    img = np.zeros((graph.height, graph.width), dtype=np.uint8)
    for r in range(graph.height):
        for c in range(graph.width):
            img[r, c] = 0 if graph.is_free(c, r) else 1

    fig, ax = plt.subplots(figsize=(12, 9))
    extent = (
        graph.origin[0],
        graph.origin[0] + graph.width * graph.resolution,
        graph.origin[1],
        graph.origin[1] + graph.height * graph.resolution,
    )
    ax.imshow(
        img,
        cmap="Greys",
        origin="lower",
        extent=extent,
        interpolation="none",
        alpha=0.6,
    )

    # Path
    xs = [graph.cell_to_world(c, r)[0] for c, r in path]
    ys = [graph.cell_to_world(c, r)[1] for c, r in path]
    ax.plot(xs, ys, "b-", linewidth=1.5, label="A* path")

    # Start / goal
    sx, sy = graph.cell_to_world(*start)
    gx, gy = graph.cell_to_world(*goal)
    ax.plot(sx, sy, "go", markersize=10, label="Start")
    ax.plot(gx, gy, "r*", markersize=12, label="Goal")

    ax.set_title(
        f"A* Path  ({graph.width}×{graph.height},"
        f" {graph.resolution:.2f} m/cell,"
        f" {len(path)} cells, {world_path_length(graph, path):.2f} m)"
    )
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.legend()
    ax.set_aspect("equal")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
