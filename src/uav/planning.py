"""Path planning: A* over the mapped occupancy grid (inflated by one cell).

The planner is *deliberately optimistic about the unknown*: unmapped cells
are treated as free. Physical safety near unmapped geometry is guaranteed by
the avoidance layer, not the planner - this separation is the point of the
architecture (see docs/01-architecture.md).
"""
import heapq
import math


def _cell(x, y, res):
    return (int(x / res), int(y / res))


def _center(cell, res):
    return ((cell[0] + 0.5) * res, (cell[1] + 0.5) * res)


def _nearest_free(cell, blocked, n, max_ring=4):
    if cell not in blocked:
        return cell
    for r in range(1, max_ring + 1):
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                c = (cell[0] + di, cell[1] + dj)
                if 0 <= c[0] < n and 0 <= c[1] < n and c not in blocked:
                    return c
    return None


def astar(blocked, start, goal, n):
    """A* on an 8-connected grid. `blocked` is a set of (i, j) cells."""
    if start == goal:
        return [start]

    def heur(c):
        dx, dy = abs(c[0] - goal[0]), abs(c[1] - goal[1])
        return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)

    open_h = [(heur(start), 0.0, start)]
    g = {start: 0.0}
    came = {}
    closed = set()
    while open_h:
        _, gc, cur = heapq.heappop(open_h)
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1]
        if cur in closed:
            continue
        closed.add(cur)
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                if di == 0 and dj == 0:
                    continue
                nb = (cur[0] + di, cur[1] + dj)
                if not (0 <= nb[0] < n and 0 <= nb[1] < n):
                    continue
                if nb in blocked:
                    continue
                step = 1.414214 if (di != 0 and dj != 0) else 1.0
                ng = gc + step
                if ng < g.get(nb, float("inf")):
                    g[nb] = ng
                    came[nb] = cur
                    heapq.heappush(open_h, (ng + heur(nb), ng, nb))
    return None


def plan_path(mapper, start_xy, goal_xy, cfg):
    """Returns a list of world-frame waypoints, or None if no path exists."""
    res = cfg.grid_res
    n = mapper.n
    blocked = mapper.blocked_cells()
    start = _nearest_free(_cell(start_xy[0], start_xy[1], res), blocked, n)
    goal = _nearest_free(_cell(goal_xy[0], goal_xy[1], res), blocked, n)
    if start is None or goal is None:
        return None
    cells = astar(blocked, start, goal, n)
    if cells is None:
        return None
    return [_center(c, res) for c in cells]
