"""Perception: fuses camera detections with CSI presence inference into a
single human-presence assessment, and maintains the probabilistic human
world model.

Nothing in this module can ever emit a *confirmed* fact about a person:
- camera detections are inferences with confidence,
- CSI produces presence probability only (it cannot localize),
- the fused assessment keeps inferred=True / confirmed=False structurally.
"""
import math
from typing import Optional

from .types import CameraObs, CsiObs, PresenceAssessment


def assess_presence(cam: Optional[CameraObs], csi: Optional[CsiObs]) -> Optional[PresenceAssessment]:
    """Fuse camera + CSI into one presence belief (0..1).

    Weighting: equal trust in the two modalities. Neither alone can push the
    fused confidence above the mission's inspect threshold, which is the
    point: cross-modal agreement is required to act.
    """
    if cam is None and csi is None:
        return None
    sources = []
    cam_belief = 0.0
    if cam is not None and cam.detections:
        cam_belief = max(d.confidence for d in cam.detections)
        sources.append("camera")
    csi_belief = 0.0
    if csi is not None:
        csi_belief = csi.inference.p_present
        sources.append("wifi_csi")
    fused = 0.5 * cam_belief + 0.5 * csi_belief
    position = None
    if cam is not None and cam.detections:
        best = max(cam.detections, key=lambda d: d.confidence)
        position = (best.x, best.y)
    return PresenceAssessment(
        assessed_present=fused > 0.5,
        confidence=fused,
        uncertainty=1.0 - fused,
        sources=sources,
        inferred=True,
        confirmed=False,
        position=position)


class HumanWorldModel:
    """Probabilistic human world model. Entries are *beliefs*, explicitly
    marked inferred; `confirmed` is always empty by construction."""

    def __init__(self):
        self.probable_humans = []  # list of dicts, one per INSPECT assessment

    def record(self, t, assessment: PresenceAssessment):
        self.probable_humans.append({
            "t": t,
            "confidence": round(assessment.confidence, 3),
            "sources": list(assessment.sources),
            "position": assessment.position,
            "inferred": True,
            "confirmed": False,   # never True - see docs/03-sensor-fusion.md
        })

    @property
    def confirmed_humans(self):
        return []  # structurally always empty: no confirmation path exists
