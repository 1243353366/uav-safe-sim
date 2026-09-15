"""Mission state machine: IDLE -> TAKEOFF -> NAV -> (INSPECT) -> RTL -> LAND -> DONE.

The mission layer *proposes* commands; the safety layer (safety.py) holds
final authority and can force RTL / LAND / LOITER at any time via
`on_override`. The mission never bypasses the safety layer - there is no
code path that reaches flight control without passing through it.

INSPECT is entered only on a *fused* presence assessment above threshold
(camera + Wi-Fi CSI agreement), and even then the recorded event is a
probable-presence belief, never a confirmed fact.
"""
import math

from .types import Command, PresenceAssessment, PoseEstimate
from .perception import HumanWorldModel
from .provenance import record_presence


class Mission:
    IDLE = "IDLE"
    TAKEOFF = "TAKEOFF"
    NAV = "NAV"
    INSPECT = "INSPECT"
    LOITER = "LOITER"
    RTL = "RTL"
    LAND = "LAND"
    DONE = "DONE"

    def __init__(self, goal, home, cfg):
        self.goal = goal
        self.home = home
        self.cfg = cfg
        self.state = Mission.TAKEOFF
        self.path = []
        self.wp_idx = 0
        self.t_state = 0.0
        self.t_last_plan = -1e9
        self.inspect_until = 0.0
        self.inspect_cooldown_until = 0.0
        self.no_path_logged = False
        self.forced_reason = None
        self.humans = HumanWorldModel()

    # ---------------- state helpers ----------------
    def _enter(self, new, t, log, note=None):
        log.event(t, "state_change", {"from": self.state, "to": new,
                                      "note": note})
        self.state = new
        self.t_state = t

    # ---------------- main step ----------------
    def step(self, t, drone, pose: PoseEstimate, mapper, presence, log):
        if self.state == Mission.DONE:
            return Command("none")

        if self.state == Mission.TAKEOFF:
            if drone.z >= self.cfg.flight_altitude - 0.3:
                self._enter(Mission.NAV, t, log, note="reached cruise altitude")
            else:
                return Command("takeoff")

        if self.state == Mission.NAV:
            return self._nav(t, drone, pose, mapper, presence, log)
        if self.state == Mission.INSPECT:
            return self._inspect(t, log)
        if self.state == Mission.LOITER:
            return Command("loiter")
        if self.state == Mission.RTL:
            return self._rtl(t, drone, pose, log)
        if self.state == Mission.LAND:
            return self._land(t, drone, log)
        return Command("loiter")

    # ---------------- sub-behaviors ----------------
    def _nav(self, t, drone, pose, mapper, presence, log):
        # fused presence belief strong enough to pause and observe
        if presence is not None and presence.assessed_present \
                and t > self.inspect_cooldown_until \
                and presence.confidence > self.cfg.presence_confirm_threshold:
            self.humans.record(t, presence)
            log.event(t, "probable_presence", {
                "confidence": round(presence.confidence, 3),
                "sources": presence.sources, "position": presence.position,
                "inferred": True, "confirmed": False})
            log.decision(record_presence(t, presence))  # provenance trail
            self._enter(Mission.INSPECT, t, log, note="probable human presence (inferred)")
            self.inspect_until = t + self.cfg.inspect_duration
            return Command("loiter")

        # replan periodically (world model keeps growing as we map)
        if not self.path or (t - self.t_last_plan) >= self.cfg.replan_period:
            self.t_last_plan = t
            from .planning import plan_path  # late import avoids cycle
            self.path = plan_path(mapper, (pose.x, pose.y), self.goal, self.cfg)
            self.wp_idx = 0
            if self.path is None:
                if not self.no_path_logged:
                    self.no_path_logged = True
                    log.event(t, "no_path", {"goal": self.goal})
                self._enter(Mission.LOITER, t, log, note="no path through known obstacles")
                return Command("loiter")

        # follow the path
        while self.wp_idx < len(self.path):
            wx, wy = self.path[self.wp_idx]
            d = math.hypot(wx - pose.x, wy - pose.y)
            if d < self.cfg.waypoint_reach_radius:
                self.wp_idx += 1
                continue
            v = self.cfg.max_speed
            return Command("velocity", vx=(wx - pose.x) / d * v,
                           vy=(wy - pose.y) / d * v)

        # path exhausted -> at (or near) goal
        if math.hypot(self.goal[0] - pose.x, self.goal[1] - pose.y) \
                < self.cfg.goal_reach_radius:
            log.event(t, "goal_reached", {"goal": self.goal})
            self._enter(Mission.RTL, t, log, note="mission complete")
            return self._rtl(t, drone, pose, log)
        return Command("loiter")  # between replans, hold briefly

    def _inspect(self, t, log):
        if t >= self.inspect_until:
            log.event(t, "inspect_done", {})
            self.inspect_cooldown_until = t + 20.0
            self._enter(Mission.NAV, t, log, note="inspection window elapsed")
            return Command("loiter")
        return Command("loiter")

    def _rtl(self, t, drone, pose, log):
        dx = self.home[0] - pose.x
        dy = self.home[1] - pose.y
        d = math.hypot(dx, dy)
        if d < self.cfg.home_reach_radius:
            log.event(t, "rtl_complete", {})
            self._enter(Mission.LAND, t, log)
            return Command("land")
        v = self.cfg.max_speed
        return Command("rtl", vx=dx / d * v, vy=dy / d * v)

    def _land(self, t, drone, log):
        if drone.landed:
            log.event(t, "mission_done", {"forced_reason": self.forced_reason})
            self._enter(Mission.DONE, t, log)
            return Command("none")
        return Command("land")

    # ---------------- safety interface ----------------
    def on_override(self, cmd: Command, t, log):
        """Called when the safety layer vetoes a proposal."""
        if cmd.kind == "rtl" and self.state not in (Mission.RTL, Mission.LAND, Mission.DONE):
            self.forced_reason = "rtl"
            self._enter(Mission.RTL, t, log, note="forced by safety layer")
        elif cmd.kind == "land" and self.state not in (Mission.LAND, Mission.DONE):
            self.forced_reason = "land"
            self._enter(Mission.LAND, t, log, note="forced by safety layer")
        elif cmd.kind == "loiter" and self.state not in (
                Mission.LOITER, Mission.RTL, Mission.LAND, Mission.DONE, Mission.INSPECT):
            self.forced_reason = "loiter"
            self._enter(Mission.LOITER, t, log, note="forced by safety layer")
