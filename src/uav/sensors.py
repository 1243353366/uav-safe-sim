"""Simulated sensors with configurable noise, degradation, and failure modes.

Each sensor mode maps to a scenario in the research protocol:
  ok       - nominal noise only
  degraded - true noise inflated but nominal sigma *reported* (silent degradation)
  drop     - no data at all (missing-data simulation)
  stale    - data frozen at an old timestamp (the timestamp is what matters)
  stuck    - values frozen but timestamp keeps advancing (implausible repeats)
  bias_jump- sudden large offset (contradicts IMU dead reckoning)
"""
import math
import random
from typing import Optional

from .types import (CameraObs, CsiObs, DepthObs, Detection, GpsObs, ImuObs,
                    PresenceInference)


def _rng(master: random.Random, k: int) -> random.Random:
    return random.Random(master.randint(0, 2**30 - 1) + k)


class GpsSensor:
    def __init__(self, mode="ok", sigma=0.5, degrade_factor=10.0,
                 jump=(40.0, 25.0), jump_at=20.0, rng=None):
        self.mode = mode
        self.sigma = sigma
        self.degrade_factor = degrade_factor
        self.jump = jump
        self.jump_at = jump_at
        self.rng = rng or random.Random(1)
        self._last: Optional[GpsObs] = None
        self._jumped = False

    def read(self, t, drone) -> Optional[GpsObs]:
        if self.mode == "drop":
            return None
        if self.mode == "stale" and self._last is not None:
            return self._last  # old timestamp: downstream staleness check fires
        true_sigma = self.sigma * (self.degrade_factor if self.mode == "degraded" else 1.0)
        x = drone.x + self.rng.gauss(0, true_sigma)
        y = drone.y + self.rng.gauss(0, true_sigma)
        if self.mode == "bias_jump" and t >= self.jump_at:
            if not self._jumped:
                self._jumped = True  # keeps contradicting IMU consistently
            x += self.jump[0]
            y += self.jump[1]
        obs = GpsObs(t=t, x=x, y=y, sigma=self.sigma)  # reports NOMINAL sigma
        self._last = obs
        return obs


class ImuSensor:
    def __init__(self, mode="ok", sigma=0.15, rng=None):
        self.mode = mode
        self.sigma = sigma
        self.rng = rng or random.Random(2)

    def read(self, t, drone) -> Optional[ImuObs]:
        if self.mode == "drop":
            return None
        return ImuObs(t=t,
                      ax=drone.ax + self.rng.gauss(0, self.sigma),
                      ay=drone.ay + self.rng.gauss(0, self.sigma))


class DepthScanner:
    """Simulated depth/LiDAR: fan of rays around the drone heading."""

    def __init__(self, mode="ok", num_rays=9, fov=2.0944, max_range=12.0,
                 sigma=0.03, rng=None):
        self.mode = mode
        self.num_rays = num_rays
        self.fov = fov
        self.max_range = max_range
        self.sigma = sigma
        self.rng = rng or random.Random(3)
        self._last: Optional[DepthObs] = None

    def read(self, t, drone, world) -> Optional[DepthObs]:
        if self.mode == "drop":
            return None
        if self.mode == "stuck" and self._last is not None:
            # values frozen, timestamp advances: detected via implausibility tests
            return DepthObs(t=t, rays=self._last.rays)
        rays = []
        for k in range(self.num_rays):
            off = -self.fov / 2 + self.fov * k / (self.num_rays - 1)
            ang = drone.yaw + off
            d = world.grid.raycast(drone.x, drone.y, ang, self.max_range)
            d = max(0.05, d + self.rng.gauss(0, self.sigma))
            rays.append((ang, d))
        obs = DepthObs(t=t, rays=rays)
        self._last = obs
        return obs


class CameraHumanDetector:
    """Simulated RGB-camera human detector.

    No CNN is executed here. This models the *statistics* of a detector
    (detection probability vs. range, false-positive rate, confidence
    distribution) so the downstream autonomy logic can be tested against
    realistic error rates. In a deployed system this is where an
    ultralytics-style model would plug in behind the same interface.

    Every detection is marked inferred=True / confirmed=False at creation.
    """

    def __init__(self, mode="ok", p_detect=0.9, p_false=0.02, fov=1.5708,
                 max_range=15.0, rng=None):
        self.mode = mode
        self.p_detect = p_detect
        self.p_false = p_false
        self.fov = fov
        self.max_range = max_range
        self.rng = rng or random.Random(4)

    def read(self, t, drone, world) -> Optional[CameraObs]:
        if self.mode == "drop":
            return None
        detections = []
        for h in world.humans:
            dx, dy = h.x - drone.x, h.y - drone.y
            dist = math.hypot(dx, dy)
            if dist > self.max_range or dist < 1e-6:
                continue
            rel = abs((math.atan2(dy, dx) - drone.yaw + math.pi) % (2 * math.pi) - math.pi)
            if rel > self.fov / 2:
                continue
            if not world.los_clear(drone.x, drone.y, h.x, h.y):
                continue
            if self.rng.random() < self.p_detect:
                conf = self.rng.uniform(0.55, 0.95) * (1 - 0.4 * dist / self.max_range)
                detections.append(Detection(
                    t=t,
                    x=h.x + self.rng.gauss(0, 0.4),
                    y=h.y + self.rng.gauss(0, 0.4),
                    confidence=conf, source="camera"))
        if self.rng.random() < self.p_false:
            d = self.rng.uniform(3.0, self.max_range)
            a = drone.yaw + self.rng.uniform(-self.fov / 2, self.fov / 2)
            detections.append(Detection(
                t=t,
                x=drone.x + math.cos(a) * d,
                y=drone.y + math.sin(a) * d,
                confidence=self.rng.uniform(0.5, 0.75), source="camera"))
        return CameraObs(t=t, detections=detections)


class WifiCsiSensor:
    """Simulated Wi-Fi CSI presence sensing - EXPERIMENTAL sensor, not ground truth.

    Pipeline stages are kept explicit and distinguishable:
      1. raw      - observed signal: noisy per-subcarrier amplitude frame
      2. feature  - processed measurement: Doppler-energy proxy from motion
                    (moving humans contribute strongly, static ones weakly)
      3. inference- inferred human presence with confidence and uncertainty

    The inference is structurally marked inferred=True / confirmed=False.
    CSI contributes presence belief only - it does NOT localize a person.
    """

    def __init__(self, mode="ok", radius=6.0, n_subcarriers=30,
                 feature_threshold=2.0, feature_scale=0.8, rng=None):
        self.mode = mode
        self.radius = radius
        self.n = n_subcarriers
        self.thr = feature_threshold
        self.scale = feature_scale
        self.rng = rng or random.Random(5)

    def read(self, t, drone, world) -> Optional[CsiObs]:
        if self.mode == "drop":
            return None
        # stage 2 basis: energy contributed by humans within sensing radius
        energy = 0.0
        for h in world.humans:
            dist = math.hypot(h.x - drone.x, h.y - drone.y)
            if dist < self.radius:
                energy += (self.radius - dist) * (1.5 if h.moving else 0.35)
        # stage 1: observed raw signal
        raw = [max(0.0, energy + self.rng.gauss(0, 0.35)) for _ in range(self.n)]
        # stage 2: processed measurement
        feature = sum(raw) / len(raw)
        # stage 3: inference (never a confirmed fact)
        p = 1.0 / (1.0 + math.exp(-(feature - self.thr) / self.scale))
        present = p > 0.5
        confidence = max(p, 1.0 - p)
        inference = PresenceInference(
            present=present, p_present=p, confidence=confidence,
            uncertainty=1.0 - confidence, inferred=True, confirmed=False)
        return CsiObs(t=t, raw=raw, feature=feature, inference=inference)
