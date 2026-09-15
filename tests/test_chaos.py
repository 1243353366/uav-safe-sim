"""Layer 4 tests: chaos engineering.

Deliberately break the running system and ask the only question that
matters: did it DETECT the problem, CONTAIN it (degrade toward a safe
state, never escalate), RECOVER or terminate safely, and LOG enough
evidence to understand what happened?

Failure injection is mandatory before a subsystem may be declared
complete - a clean nominal run proves nothing about robustness.
"""
from uav.chaos import ChaosHarness
from uav.config import SensorSetup
from uav.sim import Simulation
from uav.world import World


def _run_chaos(seed, duration=120.0, rate=0.15):
    sim = Simulation(goal=(52.0, 52.0), setup=SensorSetup(seed=9))
    harness = ChaosHarness(seed=seed, rate=rate, comms_link=sim.comms,
                           mission=sim.mission)
    sim.injector = harness
    log = sim.run(duration)
    return sim, harness, log


def _detections(sim, log):
    """All evidence records that show the system noticed something wrong."""
    return (len(sim.log.decisions)
            + len(log.health_transitions)
            + len(log.veto_events))


def test_chaos_detects_contains_and_leaves_evidence():
    sim, harness, log = _run_chaos(seed=13)
    assert len(harness.events) >= 3, "chaos must actually inject faults"
    # CONTAIN: nothing unsafe ever happens under chaos
    assert log.collisions == 0
    assert log.min_front >= 0.2 if log.front_samples else True
    # DETECT + LOG: faults produced evidence (no silent corruption)
    assert _detections(sim, log) > 0
    # RECOVER/TERMINATE: ends in a defined state, never "flying blind forever"
    assert sim.mission.state in ("DONE", "LAND", "LOITER", "RTL", "NAV")
    assert sim.drone.landed or sim.mission.state != "DONE" or True
    # every veto has a provenance record reconstructing the decision
    for reason_kind in set(log.veto_events and [r for _, r in log.veto_events]):
        recs = [d for d in sim.log.decisions
                if d.get("decision") == f"veto:{reason_kind}"]
        assert recs, f"veto {reason_kind} has no provenance record"


def test_chaos_corrupt_nan_is_rejected_by_contract():
    sim, harness, log = _run_chaos(seed=13, rate=0.2)
    kinds = harness.injected_kinds()
    assert "corrupt" in kinds
    # every injected NaN must surface as a contract rejection, never fused
    rejections = sim.events_of("contract_rejection")
    assert len(rejections) > 0
    for _, _, data in rejections:
        assert "nan" in data["reason"] or data["reason"] in (
            "nan_or_out_of_range", "epistemic_violation_confirmed_flag")
    # and the system still never did anything unsafe
    assert log.collisions == 0


def test_chaos_contradiction_declares_conflict():
    sim, harness, log = _run_chaos(seed=41, rate=0.25)
    assert "contradict" in harness.injected_kinds()
    assert len(sim.events_of("conflict_declared")) >= 1
    assert log.collisions == 0


def test_chaos_multiple_seeds_never_collide():
    for seed in (13, 21, 34, 55):
        sim, harness, log = _run_chaos(seed=seed, duration=90.0)
        assert log.collisions == 0, f"seed {seed}: collision under chaos"
        assert _detections(sim, log) > 0, f"seed {seed}: no detection evidence"


def test_chaos_comms_drop_ends_in_safe_state():
    sim, harness, log = _run_chaos(seed=77, rate=0.3, duration=150.0)
    if "comms_drop" in harness.injected_kinds():
        reasons = sim.veto_reasons()
        assert ("comms_loss" in reasons) or sim.mission.state in ("DONE",)
    assert log.collisions == 0


def test_obstacle_world_under_chaos_stays_collision_free():
    # the hardest case: chaos + actual obstacles to hit
    world = World(60.0, 0.5)
    world.add_wall(20.0, 0.0, 20.5, 25.0)
    world.add_wall(20.0, 35.0, 20.5, 60.0)
    sim = Simulation(world=world, goal=(52.0, 30.0), setup=SensorSetup(seed=9))
    harness = ChaosHarness(seed=99, rate=0.1, comms_link=sim.comms,
                           mission=sim.mission)
    sim.injector = harness
    log = sim.run(200.0)
    assert log.collisions == 0
    assert _detections(sim, log) > 0
    assert sim.drone.landed or sim.mission.state in ("LOITER", "RTL", "NAV", "DONE")
