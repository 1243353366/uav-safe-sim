"""Scenario tests: sensor failures, degradation, and contradictions.

Required cases: sensor failure (GPS/depth loss), GPS degradation,
contradictory sensor observations (GPS vs. IMU dead reckoning),
stale/missing/frozen data. All must degrade toward a safe state - never
escalate aggression.
"""
from uav.config import SensorSetup
from uav.sim import Simulation


def test_gps_drop_grows_uncertainty_and_lands_safely():
    sim = Simulation(goal=(52.0, 52.0), setup=SensorSetup(gps_mode="drop", seed=3))
    log = sim.run(70.0)
    assert "localization_lost" in sim.veto_reasons()
    assert log.collisions == 0
    assert sim.drone.landed  # degradation ended in a controlled landing
    assert any(p["sigma"] is not None and p["sigma"] > 5.0 for p in log.poses)


def test_gps_stale_is_treated_as_missing():
    sim = Simulation(goal=(52.0, 52.0), setup=SensorSetup(gps_mode="stale", seed=3))
    log = sim.run(70.0)
    assert "localization_lost" in sim.veto_reasons()
    assert log.collisions == 0
    assert sim.drone.landed


def test_gps_degraded_declares_conflict_and_lands_safely():
    # true noise is 10x worse than reported: silent degradation.
    # Once the conflicting source is excluded, dead-reckoning uncertainty
    # grows fast; the landing may be reached via localization_lost before
    # the 30 s conflict escalation - both are valid safe degradations.
    sim = Simulation(goal=(52.0, 52.0),
                     setup=SensorSetup(gps_mode="degraded", seed=3))
    log = sim.run(80.0)
    reasons = sim.veto_reasons()
    assert "sensor_conflict" in reasons
    landing_reasons = [r for r in reasons
                       if r in ("localization_lost", "conflict_escalate")]
    assert landing_reasons, "conflict must end in a safe landing, not flight"
    assert sim.localizer.health.conflict_active
    assert log.collisions == 0
    assert sim.drone.landed


def test_gps_bias_jump_is_rejected_not_fused():
    # GPS suddenly contradicts IMU dead reckoning by ~47 m
    sim = Simulation(goal=(52.0, 52.0), setup=SensorSetup(gps_mode="bias_jump", seed=3))
    log = sim.run(80.0)
    reasons = sim.veto_reasons()
    assert "sensor_conflict" in reasons
    assert any(r in ("localization_lost", "conflict_escalate") for r in reasons)
    assert log.collisions == 0
    assert sim.drone.landed
    # fused position must stay near truth, not jump with the bad GPS
    final = log.poses[-1]
    assert abs(final["x"]) < 60 and abs(final["y"]) < 60


def test_depth_drop_blinds_drone_into_loiter():
    sim = Simulation(goal=(52.0, 52.0), setup=SensorSetup(depth_mode="drop", seed=3))
    log = sim.run(30.0)
    assert "perception_lost" in sim.veto_reasons()
    assert log.collisions == 0
    last = log.poses[-1]
    assert abs(last["x"] - 5.0) < 2.5 and abs(last["y"] - 5.0) < 2.5  # stayed put


def test_depth_stuck_frozen_values_are_distrusted():
    # values frozen (fresh timestamps): caught by the plausibility check
    sim = Simulation(goal=(52.0, 52.0), setup=SensorSetup(depth_mode="stuck", seed=3))
    log = sim.run(30.0)
    assert "perception_lost" in sim.veto_reasons()
    assert log.collisions == 0


def test_imu_drop_still_flies_conservatively():
    sim = Simulation(goal=(52.0, 52.0), setup=SensorSetup(imu_mode="drop", seed=3))
    log = sim.run(80.0)
    assert log.collisions == 0
    # GPS alone still localizes; mission should complete
    assert sim.mission.state == "DONE"
