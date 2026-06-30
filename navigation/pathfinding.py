"""A* pathfinding on configuration-space grid maps with path simplification."""

import numpy as np
import heapq
from pathlib import Path

# World boundaries (must match mapping/grid_map.py)
X_MIN, X_MAX = -2.25, +2.25
Y_MIN, Y_MAX = -3.92, +1.75

# File path for saved configuration space
CSPACE_PATH = Path(__file__).resolve().parent.parent / "map_save" / "cspace.npy"


def load_cspace(path=None):
    """Load the configuration-space map from file."""
    p = Path(path) if path else CSPACE_PATH
    if not p.exists():
        return None
    return np.load(p)


def world_to_pixel(xw, yw, rows, cols):
    """Convert world coordinates to pixel indices using the same
    transform as MappingMixin.mapping()."""
    x_px = int(np.clip((xw - X_MIN) / (X_MAX - X_MIN) * cols, 0, cols - 1))
    y_px = int(np.clip((yw - Y_MIN) / (Y_MAX - Y_MIN) * rows, 0, rows - 1))
    return x_px, y_px


def pixel_to_world(x_px, y_px, rows, cols):
    """Convert pixel indices back to world coordinates."""
    xw = (x_px / cols) * (X_MAX - X_MIN) + X_MIN
    yw = (y_px / rows) * (Y_MAX - Y_MIN) + Y_MIN
    return xw, yw


def astar(cspace, start_world, goal_world):
    """A* pathfinding on a configuration-space grid map."""
    rows, cols = cspace.shape

    start_px = world_to_pixel(*start_world, rows, cols)
    goal_px = world_to_pixel(*goal_world, rows, cols)

    # Map uses (x, y) indexing convention: cspace[x, y]
    start_xy = (start_px[0], start_px[1])
    goal_xy = (goal_px[0], goal_px[1])

    # Bounds & obstacle check
    if not (0 <= start_xy[0] < rows and 0 <= start_xy[1] < cols):
        return None
    if not (0 <= goal_xy[0] < rows and 0 <= goal_xy[1] < cols):
        return None
    if cspace[start_xy] or cspace[goal_xy]:
        return None

    # If already at goal
    if start_xy == goal_xy:
        return [start_world, goal_world]

    # 8-connected neighbours with move costs
    _NEIGHBOURS = [
        (-1, -1, np.sqrt(2)),
        (-1, 0, 1.0),
        (-1, 1, np.sqrt(2)),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (1, -1, np.sqrt(2)),
        (1, 0, 1.0),
        (1, 1, np.sqrt(2)),
    ]

    def _heuristic(xy):
        dx = abs(xy[0] - goal_xy[0])
        dy = abs(xy[1] - goal_xy[1])
        return np.sqrt(dx**2 + dy**2)

    open_heap = [(0.0, start_xy)]
    came_from = {}
    g_score = {start_xy: 0.0}

    while open_heap:
        _, cur = heapq.heappop(open_heap)

        if cur == goal_xy:
            # Reconstruct pixel path in (x, y) order
            path_px = [(cur[0], cur[1])]
            while cur in came_from:
                cur = came_from[cur]
                path_px.append((cur[0], cur[1]))
            path_px.reverse()

            # Simplify then convert to world
            simple_px = simplify_path(path_px, cspace)
            return [pixel_to_world(px, py, rows, cols) for px, py in simple_px]

        for dx, dy, move_cost in _NEIGHBOURS:
            nx = cur[0] + dx
            ny = cur[1] + dy
            nb = (nx, ny)

            if not (0 <= nx < rows and 0 <= ny < cols):
                continue
            if cspace[nb]:
                continue

            tent_g = g_score[cur] + move_cost
            if tent_g < g_score.get(nb, float("inf")):
                came_from[nb] = cur
                g_score[nb] = tent_g
                heapq.heappush(open_heap, (tent_g + _heuristic(nb), nb))

    return None  # no path


def simplify_path(path_px, cspace):
    """Remove redundant waypoints that share a clear line-of-sight."""
    if len(path_px) <= 2:
        return path_px

    kept = [path_px[0]]
    anchor = 0

    for i in range(1, len(path_px) - 1):
        if not _line_of_sight(path_px[anchor], path_px[i + 1], cspace):
            kept.append(path_px[i])
            anchor = i

    kept.append(path_px[-1])
    return kept


def _line_of_sight(p1, p2, cspace):
    """Bresenham line check — False if any cell on the line is blocked."""
    x1, y1 = int(p1[0]), int(p1[1])
    x2, y2 = int(p2[0]), int(p2[1])

    rows, cols = cspace.shape
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1
    err = dx - dy

    while True:
        if not (0 <= x1 < rows and 0 <= y1 < cols):
            return False
        if cspace[x1, y1]:
            return False
        if x1 == x2 and y1 == y2:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x1 += sx
        if e2 < dx:
            err += dx
            y1 += sy

    return True
