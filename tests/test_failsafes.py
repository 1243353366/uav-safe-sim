"""Scenario tests: mission-level failsafes.

Required cases: communications loss, low battery, emergency landing /
return behavior. The safety layer must enforce each one even if the mission
logic is unaware of the condition.
"""
from uav.config import SensorSetup
from uav.sim import Simulation


def test_comms_loss_triggers_return_to_home():
    setup = SensorSetup(comms_mode="lost", comms_loss_at=5.0)  # failsafe must fire mid-NAV, not after mission RTL
    sim = Simulation(goal=(52.0, 52.0), setup=setup)
    log = sim.run(100.0)
    assert "comms_loss" in sim.veto_reasons()
    assert len(sim.events_of("rtl_complete")) == 1
    assert sim.mission.state == "DONE" and sim.drone.landed
    last = log.poses[-1]
    assert abs(last["x"] - 5.0) < 2.0 and abs(last["y"] - 5.0) < 2.0


def test_low_battery_returns_home_before_critical():
    sim = Simulation(goal=(52.0, 52.0), battery=26.5)  # crosses RTH threshold mid-NAV
    log = sim.run(100.0)
    reasons = sim.veto_reasons()
    assert "battery_rth" in reasons
    assert "battery_land" not in reasons  # made it home without a forced landing
    assert sim.mission.state == "DONE" and sim.drone.landed
    last = log.poses[-1]
    assert last["battery"] > 15.0  # never got near a dead battery
    assert abs(last["x"] - 5.0) < 2.0 and abs(last["y"] - 5.0) < 2.0


def test_critical_battery_lands_immediately_where_it_is():
    sim = Simulation(goal=(52.0, 52.0), battery=10.0)  # below land threshold
    log = sim.run(25.0)
    assert "battery_land" in sim.veto_reasons()
    assert sim.mission.state == "DONE" and sim.drone.landed
    assert sim.drone.z == 0.0


def test_uncertainty_and_low_battery_both_present_battery_wins():
    # battery is critical AND localization is dead: landing is the safe action
    sim = Simulation(goal=(52.0, 52.0), battery=8.0,
                     setup=SensorSetup(gps_mode="drop", seed=3))
    log = sim.run(30.0)
    reasons = sim.veto_reasons()
    assert "battery_land" in reasons
    assert "localization_lost" not in reasons  # battery has higher priority
    assert sim.drone.landed


def test_nominal_run_records_full_telemetry_and_no_vetoes():
    sim = Simulation(goal=(52.0, 52.0))
    log = sim.run(30.0)
    assert log.veto_events == []  # safety layer never had to intervene
    assert len(log.poses) >= 290  # 30 s at 0.1 s steps
    assert len(log.csi_inferences) >= 290  # CSI pipeline audited every tick
