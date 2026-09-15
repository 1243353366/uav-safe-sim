"""Shared data types for the autonomy pipeline.

Every stage passes explicitly typed observations downstream. In particular,
presence-related types carry `inferred` / `confirmed` flags so that an
inference can never be mistaken for a verified fact anywhere in the code.
"""
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class Command:
    """Flight-control interface command.

    kinds:
      "velocity" - track (vx, vy) in world frame [m/s]
      "rtl"      - return to home (carries velocity when known)
      "loiter"   - hold position (zero velocity)
      "land"     - descend and land at current position
      "takeoff"  - climb to flight altitude
      "none"     - motors idle / mission over
    """
    kind: str
    vx: float = 0.0
    vy: float = 0.0


# ---------------- sensor observations ----------------

@dataclass
class GpsObs:
    t: float
    x: float
    y: float
    sigma: float  # reported 1-sigma (may not match true noise under degradation)


@dataclass
class ImuObs:
    t: float
    ax: float
    ay: float


@dataclass
class DepthObs:
    t: float
    rays: List[tuple]  # [(absolute_angle_rad, distance_m), ...]


@dataclass
class Detection:
    """A human detection from the (simulated) RGB camera pipeline."""
    t: float
    x: float
    y: float
    confidence: float
    source: str
    inferred: bool = True   # camera detections are inferences, never ground truth
    confirmed: bool = False


@dataclass
class CameraObs:
    t: float
    detections: List[Detection]


@dataclass
class PresenceInference:
    """Wi-Fi CSI presence inference. This is an EXPERIMENTAL sensor output:
    `confirmed` is structurally False and `inferred` is structurally True."""
    present: bool
    p_present: float      # continuous belief in presence, 0..1
    confidence: float     # how strongly the features support the conclusion
    uncertainty: float    # 1 - confidence
    inferred: bool = True
    confirmed: bool = False


@dataclass
class CsiObs:
    """Full CSI pipeline output. The three stages are kept distinct and
    inspectable: raw signal -> processed feature -> presence inference."""
    t: float
    raw: List[float]           # stage 1: observed signal (per-subcarrier amplitudes)
    feature: float             # stage 2: processed measurement (Doppler energy)
    inference: PresenceInference  # stage 3: inferred human presence


# ---------------- fusion outputs ----------------

@dataclass
class PoseEstimate:
    x: float
    y: float
    vx: float
    vy: float
    sigma: float      # 1-sigma position uncertainty radius (m)
    conflict: bool    # sensors contradicting each other


@dataclass
class PresenceAssessment:
    """Fused camera + CSI human-presence assessment. `assessed_present` is a
    probabilistic belief, not a fact; `confirmed` is structurally False."""
    assessed_present: bool
    confidence: float
    uncertainty: float
    sources: List[str] = field(default_factory=list)
    inferred: bool = True
    confirmed: bool = False
    position: Optional[tuple] = None  # best-effort position estimate, if camera saw one
