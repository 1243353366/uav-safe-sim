"""Layer 1 tests: subsystem contracts. A violating observation must be
rejected with a reason - never partially used."""
import math

from uav.contract import (CONTRACTS, Verdict, validate_all, validate_camera,
                           validate_csi, validate_depth, validate_gps,
                           validate_imu)
from uav.types import CameraObs, CsiObs, Detection, GpsObs, ImuObs, \
    PresenceInference, DepthObs


def test_gps_contract_accepts_valid():
    v = validate_gps(GpsObs(t=1.0, x=3.0, y=4.0, sigma=0.5))
    assert v.ok and v.contract == "gps"


def test_gps_contract_rejects_nan():
    v = validate_gps(GpsObs(t=1.0, x=float("nan"), y=4.0, sigma=0.5))
    assert not v.ok and v.reason == "nan_or_out_of_range"


def test_gps_contract_rejects_missing_and_zero_sigma():
    assert validate_gps(None).reason == "missing"
    v = validate_gps(GpsObs(t=1.0, x=1.0, y=1.0, sigma=0.0))
    assert not v.ok and v.reason == "bad_sigma"


def test_imu_contract_rejects_impossible_acceleration():
    ok = validate_imu(ImuObs(t=1.0, ax=1.0, ay=-2.0))
    bad = validate_imu(ImuObs(t=1.0, ax=1e6, ay=0.0))
    assert ok.ok and not bad.ok


def test_depth_contract_rejects_nan_range():
    good = DepthObs(t=1.0, rays=[(0.0, 5.0), (0.5, 12.0)])
    bad = DepthObs(t=1.0, rays=[(0.0, float("nan"))])
    assert validate_depth(good).ok
    assert not validate_depth(bad).ok


def test_camera_contract_enforces_epistemics():
    # a detection claiming confirmed=True violates the epistemic contract
    det_bad = Detection(t=1.0, x=1.0, y=1.0, confidence=0.9,
                        source="camera", confirmed=True)
    v = validate_camera(CameraObs(t=1.0, detections=[det_bad]))
    assert not v.ok and v.reason == "epistemic_violation_confirmed_flag"
    det_ok = Detection(t=1.0, x=1.0, y=1.0, confidence=0.9, source="camera")
    assert validate_camera(CameraObs(t=1.0, detections=[det_ok])).ok
    det_nan = Detection(t=1.0, x=float("nan"), y=1.0, confidence=0.9,
                        source="camera")
    assert not validate_camera(CameraObs(t=1.0, detections=[det_nan])).ok


def test_csi_contract_rejects_bad_probability_and_confirmed_flag():
    inf_bad = PresenceInference(present=True, p_present=0.9, confidence=0.9,
                                uncertainty=0.1, confirmed=True)
    obs_bad = CsiObs(t=1.0, raw=[1.0] * 30, feature=1.0, inference=inf_bad)
    v = validate_csi(obs_bad)
    assert not v.ok and v.reason == "epistemic_violation_confirmed_flag"
    inf_nan = PresenceInference(present=True, p_present=float("nan"),
                                 confidence=0.9, uncertainty=0.1)
    obs_nan = CsiObs(t=1.0, raw=[1.0] * 30, feature=1.0, inference=inf_nan)
    assert not validate_csi(obs_nan).ok


def test_validate_all_covers_every_sensor():
    verdicts = validate_all({"gps": None, "imu": None, "depth": None,
                             "camera": None, "wifi_csi": None})
    assert set(verdicts) == {"gps", "imu", "depth", "camera", "wifi_csi"}
    assert all(v.reason == "missing" for v in verdicts.values())


def test_every_contract_declares_units_timeout_and_error_states():
    for name, c in CONTRACTS.items():
        assert c.units, f"{name} has no units"
        assert c.timeout_s is not None and c.timeout_s > 0
        assert len(c.error_states) >= 3
        assert c.version
