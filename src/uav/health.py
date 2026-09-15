"""Layer 2: Graded health states.

Binary working/broken is not enough for an autonomous system. Every sensor
carries a health state:

    HEALTHY  - fresh, contract-valid observations
    DEGRADED - data arrives but is flagged (conflict, implausible, suspect)
    STALE    - no fresh data within the freshness deadline
    FAILED   - contract violations, or no data for 3x the deadline
    UNKNOWN  - no information yet (startup)

Transitions are logged (never silently overwritten) and are visible to the
safety layer, the mission layer, and the telemetry stream alike.
"""
from enum import Enum


class HealthState(Enum):
    HEALTHY = "HEALTHY"
    DEGRADED = "DEGRADED"
    STALE = "STALE"
    FAILED = "FAILED"
    UNKNOWN = "UNKNOWN"


_ORDER = {HealthState.UNKNOWN: 0, HealthState.HEALTHY: 1, HealthState.STALE: 2,
          HealthState.DEGRADED: 3, HealthState.FAILED: 4}

_SEVERITY = {HealthState.UNKNOWN: 0, HealthState.HEALTHY: 0, HealthState.STALE: 1,
             HealthState.DEGRADED: 2, HealthState.FAILED: 3}


class SensorStatus:
    def __init__(self, name, deadline_s):
        self.name = name
        self.deadline = deadline_s
        self.state = HealthState.UNKNOWN
        self.last_reason = None
        self.last_seen = None  # timestamp of last contract-valid observation

    def update(self, t, verdict_ok, contract_reason, flagged=False, age=None):
        """Compute the new health state. `flagged` marks data that passes
        its contract but is suspicious (fusion conflict, frozen values)."""
        old = self.state
        if not verdict_ok:
            if contract_reason == "missing":
                if self.last_seen is None or (t - self.last_seen) > 3 * self.deadline:
                    new = HealthState.FAILED
                elif age is not None and age > self.deadline:
                    new = HealthState.STALE
                else:
                    new = HealthState.STALE
                reason = "no_data"
            else:
                new = HealthState.FAILED
                reason = contract_reason
        else:
            if age is not None and age > self.deadline:
                new, reason = HealthState.STALE, "stale_timestamp"
            elif flagged:
                new, reason = HealthState.DEGRADED, "flagged_by_consumer"
            else:
                new, reason = HealthState.HEALTHY, None
            self.last_seen = t
        self.state = new
        self.last_reason = reason
        return (old, new) if old != new else None


class HealthMonitor:
    """Tracks health for all five sensors; transitions are returned each
    tick so the simulation can log them (detect -> contain -> recover)."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.sensors = {
            "gps": SensorStatus("gps", cfg.gps_max_age),
            "imu": SensorStatus("imu", cfg.imu_max_age),
            "depth": SensorStatus("depth", cfg.depth_max_age),
            "camera": SensorStatus("camera", cfg.depth_max_age),
            "wifi_csi": SensorStatus("wifi_csi", cfg.gps_max_age),
        }
        self.transition_count = 0

    def step(self, t, verdicts, ages, flags):
        """verdicts: {name: Verdict}; ages: {name: float|inf}; flags: {name: bool}"""
        transitions = []
        for name, status in self.sensors.items():
            verdict = verdicts.get(name)
            if verdict is None:
                continue  # sensor not present in this tick's map
            tr = status.update(t, verdict.ok, verdict.reason,
                               flagged=flags.get(name, False),
                               age=ages.get(name))
            if tr is not None:
                self.transition_count += 1
                transitions.append({
                    "t": round(t, 2), "sensor": name,
                    "from": tr[0].value, "to": tr[1].value,
                    "reason": status.last_reason})
        return transitions

    def state_of(self, name):
        return self.sensors[name].state

    def max_severity(self):
        return max(_SEVERITY[s.state] for s in self.sensors.values())
