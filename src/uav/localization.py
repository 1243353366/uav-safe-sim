"""Localization front-end: consumes raw observations, runs the EKF, and
exposes a single pose estimate with honest uncertainty to the rest of the
stack. The localization module never hides a conflict - it propagates it.
"""
from .fusion import EkfFusion
from .types import PoseEstimate


class Localizer:
    def __init__(self, cfg, x0, y0):
        self.cfg = cfg
        self.fusion = EkfFusion(x0, y0, cfg)

    def step(self, t, gps_obs, imu_obs):
        self.fusion.predict(t, imu_obs)
        self.fusion.update_gps(t, gps_obs)

    def estimate(self) -> PoseEstimate:
        return self.fusion.estimate()

    @property
    def health(self):
        return self.fusion.health
