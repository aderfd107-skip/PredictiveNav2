"""A* path planner on a 2-D occupancy grid.

Usage::

    from inspection_core.grid_graph import GridGraph
    from inspection_core.astar import astar, cell_path_to_world_path

    graph = GridGraph("maps/office_service_mvp.yaml", inflation_radius_m=0.3)
    path = astar(graph, (10, 20), (200, 150))
    waypoints = cell_path_to_world_path(graph, path, step=10)
"""

from __future__ import annotations

import heapq
import math
from typing import Dict, List, Optional, Tuple

from .grid_graph import Cell, GridGraph, WorldPoint


def astar(
    graph: GridGraph,
    start: Cell,
    goal: Cell,
    max_expansions: int = 200_000,
    snap_goal: bool = True,
    snap_start: bool = False,
) -> List[Cell] | None:
    """Return the shortest cell-path from *start* to *goal*, or *None*.

    Uses the standard A* algorithm with the graph's built-in 4-connectivity
    and Manhattan heuristic.

    Parameters
    ----------
    graph:
        The occupancy grid to search on.
    start:
        Starting cell ``(col, row)``.
    goal:
        Target cell ``(col, row)``.
    max_expansions:
        Safety limit — abort if more than this many nodes are expanded.
    snap_goal:
        If True and *goal* is occupied, snap to the nearest free cell.
    snap_start:
        If True and *start* is occupied, snap to the nearest free cell.
    """
    if snap_start and not graph.is_free(*start):
        snapped = graph.snap_to_free(*start)
        if snapped is None:
            return None
        start = snapped

    if not graph.is_free(*start):
        return None

    if snap_goal and not graph.is_free(*goal):
        snapped = graph.snap_to_free(*goal)
        if snapped is None:
            return None
        goal = snapped

    if not graph.is_free(*goal):
        return None

    if start == goal:
        return [start]

    # f = g + h
    g_score: Dict[Cell, float] = {start: 0.0}
    came_from: Dict[Cell, Cell] = {}

    open_set: List[Tuple[float, int, Cell]] = []
    tie = 0  # tie-breaker so two cells with equal f are stable
    heapq.heappush(
        open_set,
        (graph.heuristic(start, goal), tie, start),
    )

    expanded = 0
    while open_set:
        expanded += 1
        if expanded > max_expansions:
            return None

        f_val, _, current = heapq.heappop(open_set)

        if current == goal:
            return _reconstruct_path(came_from, current)

        current_g = g_score[current]

        for nb in graph.neighbours(*current):
            tentative_g = current_g + graph.cost(current, nb)

            if tentative_g < g_score.get(nb, math.inf):
                came_from[nb] = current
                g_score[nb] = tentative_g
                tie += 1
                heapq.heappush(
                    open_set,
                    (tentative_g + graph.heuristic(nb, goal), tie, nb),
                )

    return None  # no path found


# ---------------------------------------------------------------------------
# path helpers
# ---------------------------------------------------------------------------


def cell_path_to_world_path(
    graph: GridGraph,
    cell_path: List[Cell],
    step: int = 1,
) -> List[WorldPoint]:
    """Convert a list of *cells* into world *(x, y)* waypoints.

    *step* controls down-sampling: only every *step*-th cell is emitted
    (the first and last cells are always included).  This reduces the
    number of waypoints sent to Nav2 without sacrificing coverage.
    """
    if not cell_path:
        return []

    waypoints: List[WorldPoint] = []
    n = len(cell_path)
    for i, cell in enumerate(cell_path):
        if i == 0 or i == n - 1 or i % step == 0:
            waypoints.append(graph.cell_to_world(*cell))
    return waypoints


def cell_path_length(cell_path: List[Cell]) -> float:
    """Number of edges in the cell path (each edge = 1 cell width)."""
    return float(len(cell_path) - 1) if len(cell_path) > 1 else 0.0


def world_path_length(
    graph: GridGraph, cell_path: List[Cell]
) -> float:
    """Total world distance along *cell_path* in metres."""
    return cell_path_length(cell_path) * graph.resolution


# ---------------------------------------------------------------------------
# internal
# ---------------------------------------------------------------------------


def _reconstruct_path(
    came_from: Dict[Cell, Cell],
    current: Cell,
) -> List[Cell]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path
