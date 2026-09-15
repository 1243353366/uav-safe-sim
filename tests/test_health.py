"""Layer 2 tests: graded health states (HEALTHY/DEGRADED/STALE/FAILED/UNKNOWN)."""
from uav.config import SimConfig
from uav.contract import Verdict
from uav.health import HealthMonitor, HealthState


def _ok(name):
    return Verdict(True, name)


def _bad(name, reason="nan_or_out_of_range"):
    return Verdict(False, name, reason)


def test_startup_state_is_unknown():
    mon = HealthMonitor(SimConfig())
    for name in mon.sensors:
        assert mon.state_of(name) == HealthState.UNKNOWN


def test_missing_data_degrades_to_stale_then_failed():
    mon = HealthMonitor(SimConfig())
    cfg = SimConfig()
    # valid data at t=0 -> HEALTHY
    trs = mon.step(0.0, {"gps": _ok("gps")}, {"gps": 0.0}, {})
    assert mon.state_of("gps") == HealthState.HEALTHY
    # silence: first ticks -> STALE, after 3x deadline -> FAILED
    t = 0.1
    seen_stale = False
    while t < 3 * cfg.gps_max_age + 1.0:
        mon.step(t, {"gps": Verdict(False, "gps", "missing")},
                 {"gps": float("inf")}, {})
        if mon.state_of("gps") == HealthState.STALE:
            seen_stale = True
        if mon.state_of("gps") == HealthState.FAILED:
            break
        t += 0.1
    assert seen_stale
    assert mon.state_of("gps") == HealthState.FAILED


def test_contract_violation_is_immediate_failure():
    mon = HealthMonitor(SimConfig())
    mon.step(0.0, {"imu": _ok("imu")}, {"imu": 0.0}, {})
    assert mon.state_of("imu") == HealthState.HEALTHY
    mon.step(0.1, {"imu": _bad("imu")}, {"imu": 0.0}, {})
    assert mon.state_of("imu") == HealthState.FAILED


def test_flagged_but_valid_data_is_degraded_not_failed():
    mon = HealthMonitor(SimConfig())
    mon.step(0.0, {"gps": _ok("gps")}, {"gps": 0.0}, {"gps": True})
    assert mon.state_of("gps") == HealthState.DEGRADED


def test_transitions_are_reported_exactly_once():
    mon = HealthMonitor(SimConfig())
    trs = mon.step(0.0, {"gps": _ok("gps")}, {"gps": 0.0}, {})
    assert len(trs) == 1  # UNKNOWN -> HEALTHY
    trs = mon.step(0.1, {"gps": _ok("gps")}, {"gps": 0.0}, {})
    assert len(trs) == 0  # no change, no report (no silent overwrite, no spam)
    trs = mon.step(0.2, {"gps": _ok("gps")}, {"gps": 0.0}, {"gps": True})
    assert trs[0]["to"] == "DEGRADED"
