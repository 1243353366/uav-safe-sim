"""Obstacle avoidance: a reflex layer between mission commands and flight
control. It guarantees the commanded speed always remains within the
current stopping distance of whatever the depth sensor sees ahead.

Fail-safe semantics: missing or stale depth data produces a full stop, not
a blind continuation. This layer cannot override failsafe commands
(land/loiter/takeoff) - it only scales down motion.
"""
import math

from .types import Command, DepthObs
from .world import wrap_angle


class AvoidanceLayer:
    def __init__(self, cfg):
        self.cfg = cfg

    def adjust(self, cmd: Command, depth: DepthObs, t: float) -> Command:
        if cmd.kind not in ("velocity", "rtl"):
            return cmd  # loiter/land/takeoff carry no lateral motion to gate
        v = math.hypot(cmd.vx, cmd.vy)
        if v < 1e-6:
            return cmd
        # Missing or stale depth data => degrade to a stop, never guess.
        if depth is None or (t - depth.t) > self.cfg.depth_max_age:
            return Command(kind=cmd.kind, vx=0.0, vy=0.0)
        heading = math.atan2(cmd.vy, cmd.vx)
        d_front = math.inf
        for ang, d in depth.rays:
            if abs(wrap_angle(ang - heading)) <= self.cfg.avoid_cone:
                d_front = min(d_front, d)
        if d_front is math.inf:
            return cmd
        margin = self.cfg.obstacle_stop_margin
        if d_front <= margin:
            return Command(kind=cmd.kind, vx=0.0, vy=0.0)
        allowed = math.sqrt(2.0 * self.cfg.max_decel * (d_front - margin))
        scale = min(1.0, allowed / v)
        return Command(kind=cmd.kind, vx=cmd.vx * scale, vy=cmd.vy * scale)
