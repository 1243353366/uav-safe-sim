"""SAFETY LAYER - independent, with FINAL authority over flight.

This module is deliberately separate from the mission logic and can veto
any mission command. It is written as a pure function of observable state so
it can be audited, tested, and mirrored in a memory-safe language (see
components/rust/safety_veto.rs) without touching mission code.

Priority order (first match wins):
  1. battery <= battery_land              -> LAND immediately
  2. battery <= battery_rth               -> RTL
  3. no pose / uncertainty > max          -> LAND (cannot trust localization)
  4. sensor conflict                      -> LOITER, escalate to LAND if persistent
  5. comms lost > comms_loss_rtl_after    -> RTL
  6. depth data stale/missing (airborne)  -> LOITER (cannot sense obstacles)
  7. geofence breach                      -> RTL

Core principle: stale, missing, contradictory, or low-confidence inputs
degrade the system toward a predefined safe state. No failure mode in this
list ever produces *more* aggressive behavior.
"""
import math

from .types import Command, PoseEstimate
from .fusion import SensorHealth


class SafetyLayer:
    def __init__(self, cfg, home, world_size):
        self.cfg = cfg
        self.home = home
        self.world_size = world_size
        self.veto_count = 0
        self._last_reason = None

    def filter(self, proposed: Command, t: float, drone, pose: PoseEstimate,
               health: SensorHealth, depth_age: float, comms_lost_for: float,
               log):
        """Returns (allowed_command, veto_reason_or_None). The reason feeds
        the provenance layer; the command feeds flight control."""
        # Once the aircraft is on the ground, all failsafes are moot.
        if drone.landed and drone.z <= 0.0:
            return proposed, None

        """Returns the command actually allowed to execute. Veto events are
        logged with a reason string; identical compliant commands pass
        through untouched so the veto log records real interventions only."""

        def override(cmd: Command, reason: str):
            if cmd.kind == proposed.kind:
                return proposed, None  # mission already complying; not a veto
            # log the first veto of each consecutive run, not every tick
            if reason != self._last_reason:
                self.veto_count += 1
                log.veto(t, reason)
                self._last_reason = reason
            return cmd, reason

        # 1. critical battery
        if drone.battery <= self.cfg.battery_land:
            return override(Command("land"), "battery_land")
        # 2. reserve battery - never interrupt an active landing
        if drone.battery <= self.cfg.battery_rth \
                and proposed.kind not in ("land", "none"):
            return override(self._rtl_cmd(pose), "battery_rth")
        # 3. localization untrustworthy
        if pose is None or pose.sigma > self.cfg.max_uncertainty:
            return override(Command("land"), "localization_lost")
        # 4. sensors contradicting each other
        if health.conflict_active:
            if (health.conflict_t0 is not None
                    and (t - health.conflict_t0) > self.cfg.conflict_loiter_max):
                return override(Command("land"), "conflict_escalate")
            return override(Command("loiter"), "sensor_conflict")
        # 5. lost link - force RTL, but never interrupt an active landing:
        #    landing at/near home already satisfies the failsafe intent.
        if comms_lost_for > self.cfg.comms_loss_rtl_after \
                and proposed.kind not in ("land", "none"):
            return override(self._rtl_cmd(pose), "comms_loss")
        # 6. blind while airborne (no trustworthy obstacle sensing)
        if depth_age > self.cfg.depth_max_age and drone.motors_on:
            return override(Command("loiter"), "perception_lost")
        # 7. geofence
        m = self.cfg.geofence_margin
        if not (m <= drone.x <= self.world_size - m
                and m <= drone.y <= self.world_size - m) \
                and proposed.kind not in ("land", "none"):
            return override(self._rtl_cmd(pose), "geofence_breach")
        return proposed, None

    def _rtl_cmd(self, pose: PoseEstimate) -> Command:
        if pose is None:
            return Command("rtl")  # hover-ish; localization failure handled above
        dx = self.home[0] - pose.x
        dy = self.home[1] - pose.y
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return Command("rtl")
        v = self.cfg.max_speed
        return Command("rtl", vx=dx / d * v, vy=dy / d * v)
