"""Scenario tests: waypoint navigation and obstacle avoidance (with mapping)."""
from uav.config import SensorSetup
from uav.sim import Simulation
from uav.world import World


def test_waypoint_navigation_nominal():
    sim = Simulation(goal=(52.0, 52.0))
    log = sim.run(80.0)
    assert sim.mission.state == "DONE"
    assert log.collisions == 0
    assert log.veto_events == []  # nominal run: safety layer never intervenes
    assert len(sim.events_of("goal_reached")) == 1
    assert len(sim.events_of("rtl_complete")) == 1
    last = log.poses[-1]
    assert last["z"] == 0.0  # landed
    assert abs(last["x"] - 5.0) < 2.0 and abs(last["y"] - 5.0) < 2.0


def test_obstacle_avoidance_routes_through_gap_and_never_collides():
    # wall with a gap: x in [20, 20.5], y in [0, 25] and [35, 60]
    world = World(60.0, 0.5)
    world.add_wall(20.0, 0.0, 20.5, 25.0)
    world.add_wall(20.0, 35.0, 20.5, 60.0)
    sim = Simulation(world=world, goal=(52.0, 52.0))
    log = sim.run(180.0)

    assert log.collisions == 0
    assert log.min_front >= 0.25  # never got dangerously close to anything
    assert len(sim.events_of("goal_reached")) == 1

    # the drone must pass the wall through the gap (y in (25, 35)), not over/through
    crossings = []
    prev = log.poses[0]
    for p in log.poses[1:]:
        if prev["x"] < 20.0 <= p["x"]:
            crossings.append(p)
        prev = p
    assert crossings, "drone never crossed the wall line"
    for p in crossings:
        assert 24.5 < p["y"] < 35.5, f"crossed wall outside gap at y={p['y']}"


def test_obstacle_avoidance_brakes_when_path_blocked_by_unknown_wall():
    # solid wall across the direct route; only route is around the top edge
    world = World(60.0, 0.5)
    world.add_wall(20.0, 0.0, 20.5, 40.0)
    sim = Simulation(world=world, goal=(52.0, 30.0))
    log = sim.run(200.0)
    assert log.collisions == 0
    assert log.min_front >= 0.25
    assert len(sim.events_of("goal_reached")) == 1
    # every crossing of x=20 must be above the wall (y > 40)
    prev = log.poses[0]
    for p in log.poses[1:]:
        if prev["x"] < 20.0 <= p["x"]:
            assert p["y"] > 39.5, f"crossed wall body at y={p['y']}"
        prev = p
