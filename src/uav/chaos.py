"""Layer 4: Chaos testing.

Once the basic simulation works, deliberately break it. The ChaosHarness
sits between the sensors and the fusion layer (like a flaky network) and
injects, on a schedule driven by a seeded RNG:

  drop        - a sensor message simply never arrives (missing data)
  corrupt     - a value is replaced with NaN (malformed measurement)
  delay       - the timestamp is stale by several seconds (late arrival)
  duplicate   - the previous observation is replayed (idempotency test)
  contradict  - a large position offset is injected (contradictory sensors)
  comms_drop  - the ground link goes away mid-mission
  restart     - the mission state machine is bounced to NAV mid-flight

The question every chaos run must answer: did the system DETECT the
problem, CONTAIN it (degrade toward a safe state, never escalate), RECOVER
or terminate safely, and LOG enough evidence to understand what happened?

Failure injection is mandatory before a subsystem may be declared complete
(see docs/06-open-questions.md and the test suite).
"""
import math
import random
from dataclasses import dataclass, field
from typing import Optional

from .types import DepthObs, GpsObs, ImuObs


@dataclass
class ChaosEvent:
    t: float
    kind: str
    target: str
    detail: dict = field(default_factory=dict)


class ChaosHarness:
    """Intercepts the observation map each tick and may mangle it."""

    def __init__(self, seed=13, rate=0.01, kinds=None, comms_link=None,
                 mission=None, duration_hint=120.0):
        self.rng = random.Random(seed)
        self.rate = rate                      # expected faults per second
        self.kinds = kinds or ["drop", "corrupt", "delay", "duplicate",
                               "contradict"]
        self.comms_link = comms_link
        self.mission = mission
        self.duration_hint = duration_hint
        self.events = []
        self._active = {}  # target -> (kind, until_t)
        self._next_event_t = 0.0
        self._rng_pick_next()

    def _rng_pick_next(self):
        # exponentially distributed inter-arrival, bounded
        self._next_event_t += min(30.0, self.rng.expovariate(self.rate))

    def _schedule(self, t):
        while self._next_event_t <= t:
            kind = self.rng.choice(self.kinds)
            target = self.rng.choice(["gps", "imu", "depth", "camera", "wifi_csi"])
            if kind == "comms_drop":
                target = "comms"
            window = round(self.rng.uniform(0.5, 4.0), 2)
            self.events.append(ChaosEvent(
                t=round(self._next_event_t, 2), kind=kind, target=target,
                detail={"until": round(self._next_event_t + window, 2)}))
            if kind == "comms_drop" and self.comms_link is not None:
                self.comms_link.mode = "lost"
                self.comms_link.loss_at = self._next_event_t
            elif kind == "restart" and self.mission is not None:
                self.mission.state = "NAV"  # bounced mid-flight
                self.mission.t_state = self._next_event_t
            else:
                self._active[target] = (kind, self._next_event_t + window)
            self._rng_pick_next()

    def intercept(self, t, obs_map):
        """Returns (possibly mangled) obs_map. Runs BEFORE contract
        validation, so malformed injected data must be caught downstream."""
        self._schedule(t)
        for target, (kind, until) in list(self._active.items()):
            if t > until:
                del self._active[target]
                continue
            obs = obs_map.get(target)
            if kind == "drop":
                obs_map[target] = None
            elif obs is None:
                continue
            elif kind == "corrupt":
                obs_map[target] = self._corrupt(target, obs)
            elif kind == "delay":
                obs_map[target] = self._delay(target, obs, t)
            elif kind == "duplicate":
                obs_map[target] = self._last.get(target, obs) if hasattr(self, "_last") else obs
            elif kind == "contradict":
                obs_map[target] = self._contradict(target, obs)
        self._last = dict(obs_map)
        return obs_map

    # -- individual mangling operations --
    def _corrupt(self, target, obs):
        if isinstance(obs, GpsObs):
            return GpsObs(t=obs.t, x=float("nan"), y=obs.y, sigma=obs.sigma)
        if isinstance(obs, ImuObs):
            return ImuObs(t=obs.t, ax=float("nan"), ay=obs.ay)
        if isinstance(obs, DepthObs):
            rays = [(a, float("nan")) for a, _ in obs.rays]
            return DepthObs(t=obs.t, rays=rays)
        # camera/csi: drop raw numeric payload via a bad confidence/feature
        if hasattr(obs, "inference"):  # CsiObs
            object.__setattr__(obs.inference, "p_present", float("nan"))
            return obs
        return None  # unknown shape: treat as dropped (conservative)

    def _delay(self, target, obs, t):
        stale_t = t - 5.0  # well beyond every freshness deadline
        if isinstance(obs, GpsObs):
            return GpsObs(t=stale_t, x=obs.x, y=obs.y, sigma=obs.sigma)
        if isinstance(obs, ImuObs):
            return ImuObs(t=stale_t, ax=obs.ax, ay=obs.ay)
        if isinstance(obs, DepthObs):
            return DepthObs(t=stale_t, rays=obs.rays)
        return obs

    def _contradict(self, target, obs):
        if isinstance(obs, GpsObs):
            # plausible-looking but badly wrong: must be caught by the gate
            return GpsObs(t=obs.t, x=obs.x + 35.0, y=obs.y - 20.0, sigma=obs.sigma)
        return obs

    def injected_kinds(self):
        return sorted({e.kind for e in self.events})
