"""Unit tests for the A* planner and occupancy mapping."""
from uav.config import SimConfig
from uav.mapping import OccupancyMapper
from uav.planning import astar, plan_path


def test_astar_open_grid():
    path = astar(set(), (0, 0), (10, 10), 120)
    assert path is not None
    assert path[0] == (0, 0) and path[-1] == (10, 10)


def test_astar_returns_none_when_walled_off():
    n = 120
    blocked = {(10, j) for j in range(n)}
    assert astar(blocked, (0, 0), (20, 0), n) is None


def test_astar_routes_through_gap():
    n = 120
    blocked = {(10, j) for j in range(n) if not (30 <= j <= 34)}
    path = astar(blocked, (0, 20), (20, 20), n)
    assert path is not None
    crossing = [c for c in path if c[0] == 10]
    assert crossing and all(30 <= c[1] <= 34 for c in crossing)


def test_mapper_marks_hits_and_inflates():
    m = OccupancyMapper(60.0, 0.5)
    m.occ.add((30, 30))
    blocked = m.blocked_cells()
    assert (30, 30) in blocked
    assert (31, 31) in blocked  # inflation ring
    assert (35, 35) not in blocked


def test_plan_path_returns_none_when_goal_unreachable():
    cfg = SimConfig()
    m = OccupancyMapper(cfg.world_size, cfg.grid_res)
    # a full wall of known obstacles between start and goal, no gap
    for j in range(m.n):
        m.occ.add((30, j))
    assert plan_path(m, (10.0, 10.0), (50.5, 50.5), cfg) is None


def test_plan_path_blocked_goal_relocates_to_nearest_free():
    # a planner that cannot reach the exact goal cell plans to the nearest
    # reachable cell instead of failing silently (documented behavior)
    cfg = SimConfig()
    m = OccupancyMapper(cfg.world_size, cfg.grid_res)
    for di in range(-2, 3):
        for dj in range(-2, 3):
            if abs(di) == 2 or abs(dj) == 2:
                m.occ.add((50 + di, 50 + dj))
    path = plan_path(m, (10.0, 10.0), (50.5, 50.5), cfg)
    assert path is not None and path[-1] != (50.5, 50.5)
