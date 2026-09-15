"""Layer 3: Observability / provenance.

Every significant decision must be reconstructable after the fact:

  What did the system observe?  -> inputs summary
  What time was it?             -> t (simulation clock, one monotonic clock)
  Which model processed it?     -> model_version
  What uncertainty existed?    -> uncertainty
  Which component consumed it? -> consumer
  What decision resulted?       -> decision + rationale

DecisionRecord objects are appended to the SimLog by the fusion, safety,
mission, and chaos layers. They are the evidence trail that chaos testing
asks for: enough to understand what happened, not just that it happened.
"""
from dataclasses import dataclass, field
from typing import Optional

# Model registry: pin model + preprocessing + runtime versions together.
# Any inference message that reaches the log carries the version that
# produced it, so incompatible combinations can be detected later.
MODEL_VERSIONS = {
    "camera_detector": "sim-camera-v0.1",     # simulated detector statistics
    "csi_pipeline": "sim-csi-v0.1",          # presence inference pipeline
    "fusion": "ekf-gated-v0.1",              # gated EKF
    "planner": "astar-grid-v0.1",            # A* over inflated occupancy
    "safety": "safety-veto-v0.1",             # independent veto authority
    "mission": "fsm-nav-v0.1",               # mission state machine
    "contracts": "contracts-v0.1",
}


@dataclass
class DecisionRecord:
    t: float
    component: str                  # who decided
    decision: str                   # what was decided
    inputs: dict = field(default_factory=dict)   # what it observed (summary)
    uncertainty: Optional[float] = None          # what uncertainty existed
    model_version: Optional[str] = None          # which model processed it
    consumer: Optional[str] = None               # which component consumed it
    rationale: Optional[str] = None              # why

    def as_dict(self):
        return {
            "t": round(self.t, 2), "component": self.component,
            "decision": self.decision, "inputs": self.inputs,
            "uncertainty": (round(self.uncertainty, 3)
                            if self.uncertainty is not None else None),
            "model_version": self.model_version,
            "consumer": self.consumer, "rationale": self.rationale,
        }


def record_veto(t, proposed_kind, effective_kind, reason, pose_sigma, consumer):
    """Provenance for a safety-layer intervention."""
    return DecisionRecord(
        t=t, component="safety",
        decision=f"veto:{reason}",
        inputs={"proposed": proposed_kind, "effective": effective_kind,
                "pose_sigma_m": round(pose_sigma, 3) if pose_sigma else None},
        model_version=MODEL_VERSIONS["safety"],
        consumer=consumer,
        rationale=f"proposed '{proposed_kind}' replaced by '{effective_kind}'")


def record_contract_rejection(t, sensor, reason):
    return DecisionRecord(
        t=t, component="contracts", decision=f"reject:{sensor}",
        inputs={"reason": reason},
        model_version=MODEL_VERSIONS["contracts"],
        consumer="sensor_fusion",
        rationale="observation violated its subsystem contract; dropped")


def record_health_transition(transition):
    return DecisionRecord(
        t=transition["t"], component="health",
        decision=f"{transition['sensor']}:{transition['from']}->{transition['to']}",
        inputs={"reason": transition["reason"]},
        consumer="safety_layer")


def record_conflict(t, source, detail):
    return DecisionRecord(
        t=t, component="fusion", decision="conflict_declared",
        inputs={"source": source, "detail": detail},
        model_version=MODEL_VERSIONS["fusion"],
        consumer="safety_layer",
        rationale="innovation gate rejected the source repeatedly; excluded")


def record_presence(t, assessment):
    return DecisionRecord(
        t=t, component="perception", decision="probable_presence",
        inputs={"confidence": round(assessment.confidence, 3),
                "sources": assessment.sources,
                "position": assessment.position,
                "inferred": True, "confirmed": False},
        uncertainty=assessment.uncertainty,
        model_version=f"{MODEL_VERSIONS['camera_detector']}+{MODEL_VERSIONS['csi_pipeline']}",
        consumer="mission",
        rationale="cross-modal agreement above inspect threshold; belief, not fact")
