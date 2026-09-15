"""Layer 1: Subsystem contracts.

Every subsystem boundary gets an explicit contract: input/output types,
units, valid ranges, timestamps, freshness deadlines, error states, and
version. Observations are validated against their contract before any
consumer sees them. A message that violates its contract is DROPPED and
recorded - never "mostly used".

These contracts are the single source of truth shared by every language
port (see docs/07-language-assignments.md): a Rust or C++ component must
accept the same units, ranges, and semantics defined here.
"""
import math
from dataclasses import dataclass, field
from typing import Optional

VERSION = "contracts-v0.1"

WORLD_MAX = 1e4        # physically implausible beyond this (position, m)
ACCEL_MAX = 100.0      # m/s^2: beyond any UAV
RATE_MAX = 100.0       # 1/s for any rate-like quantity


@dataclass(frozen=True)
class Contract:
    name: str
    kind: str                      # observation type this contract governs
    units: dict                    # field -> unit symbol
    valid_range: dict               # field -> (lo, hi)
    timeout_s: float               # freshness deadline (None = event-driven)
    error_states: tuple            # named ways this input can be wrong
    version: str = VERSION


CONTRACTS = {
    "gps": Contract(
        name="gps", kind="GpsObs",
        units={"x": "m", "y": "m", "sigma": "m"},
        valid_range={"x": (-WORLD_MAX, WORLD_MAX), "y": (-WORLD_MAX, WORLD_MAX),
                     "sigma": (0.0, WORLD_MAX)},
        timeout_s=2.0,
        error_states=("missing", "stale", "nan", "out_of_range", "contradicts_imu")),
    "imu": Contract(
        name="imu", kind="ImuObs",
        units={"ax": "m/s^2", "ay": "m/s^2"},
        valid_range={"ax": (-ACCEL_MAX, ACCEL_MAX), "ay": (-ACCEL_MAX, ACCEL_MAX)},
        timeout_s=0.5,
        error_states=("missing", "stale", "nan", "out_of_range")),
    "depth": Contract(
        name="depth", kind="DepthObs",
        units={"angle": "rad", "distance": "m"},
        valid_range={"distance": (0.0, 100.0)},
        timeout_s=1.0,
        error_states=("missing", "stale", "nan", "out_of_range", "frozen_values")),
    "camera": Contract(
        name="camera", kind="CameraObs",
        units={"x": "m", "y": "m", "confidence": "probability"},
        valid_range={"x": (-WORLD_MAX, WORLD_MAX), "y": (-WORLD_MAX, WORLD_MAX),
                     "confidence": (0.0, 1.0)},
        timeout_s=1.0,
        error_states=("missing", "stale", "nan", "out_of_range",
                      "false_positive", "false_negative")),
    "wifi_csi": Contract(
        name="wifi_csi", kind="CsiObs",
        units={"raw": "dB", "feature": "dB", "p_present": "probability"},
        valid_range={"raw": (0.0, WORLD_MAX), "feature": (-WORLD_MAX, WORLD_MAX),
                     "p_present": (0.0, 1.0), "confidence": (0.0, 1.0)},
        timeout_s=2.0,
        error_states=("missing", "stale", "nan", "out_of_range")),
}


@dataclass
class Verdict:
    ok: bool
    contract: str
    reason: Optional[str] = None
    version: str = VERSION

    def __str__(self):
        return (f"contract={self.contract} ok={self.ok}"
                + (f" reason={self.reason}" if self.reason else ""))


def _finite(v) -> bool:
    return isinstance(v, (int, float)) and math.isfinite(v)


def _check(value, lo, hi):
    return _finite(value) and lo <= value <= hi


def validate_gps(obs) -> Verdict:
    c = CONTRACTS["gps"]
    if obs is None:
        return Verdict(False, c.name, "missing")
    if not (_check(obs.x, *c.valid_range["x"]) and _check(obs.y, *c.valid_range["y"])):
        return Verdict(False, c.name, "nan_or_out_of_range")
    if not _check(obs.sigma, *c.valid_range["sigma"]) or obs.sigma <= 0:
        return Verdict(False, c.name, "bad_sigma")
    return Verdict(True, c.name)


def validate_imu(obs) -> Verdict:
    c = CONTRACTS["imu"]
    if obs is None:
        return Verdict(False, c.name, "missing")
    if not (_check(obs.ax, *c.valid_range["ax"]) and _check(obs.ay, *c.valid_range["ay"])):
        return Verdict(False, c.name, "nan_or_out_of_range")
    return Verdict(True, c.name)


def validate_depth(obs) -> Verdict:
    c = CONTRACTS["depth"]
    if obs is None:
        return Verdict(False, c.name, "missing")
    lo, hi = c.valid_range["distance"]
    for _, d in obs.rays:
        if not _check(d, lo, hi):
            return Verdict(False, c.name, "nan_or_out_of_range")
    return Verdict(True, c.name)


def validate_camera(obs) -> Verdict:
    c = CONTRACTS["camera"]
    if obs is None:
        return Verdict(False, c.name, "missing")
    for det in obs.detections:
        if not (_check(det.x, *c.valid_range["x"])
                and _check(det.y, *c.valid_range["y"])
                and _check(det.confidence, *c.valid_range["confidence"])):
            return Verdict(False, c.name, "nan_or_out_of_range")
        if det.confirmed:  # structural: a camera detection can never be confirmed
            return Verdict(False, c.name, "epistemic_violation_confirmed_flag")
    return Verdict(True, c.name)


def validate_csi(obs) -> Verdict:
    c = CONTRACTS["wifi_csi"]
    if obs is None:
        return Verdict(False, c.name, "missing")
    inf = obs.inference
    if not _check(inf.p_present, 0.0, 1.0) or not _check(inf.confidence, 0.0, 1.0):
        return Verdict(False, c.name, "nan_or_out_of_range")
    if inf.confirmed or not inf.inferred:  # CSI can never assert a confirmed fact
        return Verdict(False, c.name, "epistemic_violation_confirmed_flag")
    for v in obs.raw:
        if not _check(v, *c.valid_range["raw"]):
            return Verdict(False, c.name, "nan_or_out_of_range")
    return Verdict(True, c.name)


VALIDATORS = {"gps": validate_gps, "imu": validate_imu, "depth": validate_depth,
              "camera": validate_camera, "wifi_csi": validate_csi}


def validate_all(obs_map) -> dict:
    """Validate every observation at the subsystem boundary.
    Returns {name: Verdict}. Violations are dropped by the caller and
    recorded as provenance (see provenance.py)."""
    return {name: VALIDATORS[name](obs) for name, obs in obs_map.items()}
