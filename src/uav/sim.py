"""Simulation harness: wires sensors -> fusion -> mapping -> mission ->
safety -> avoidance -> flight control, and records everything for analysis.

This is the executable integration point of the architecture. It is also the
reference for the ROS 2 / PX4 SITL port: each block here maps onto a ROS 2
node (see docs/05-simulation-setup.md).
"""
import random

from .avoidance import AvoidanceLayer
from .comms import CommsLink
from .contract import validate_all
from .health import HealthMonitor
from .provenance import record_conflict, record_contract_rejection, \
    record_health_transition, record_veto
from .config import SimConfig, SensorSetup
from .localization import Localizer
from .mapping import OccupancyMapper
from .mission import Mission
from .perception import assess_presence
from .safety import SafetyLayer
from .sensors import (CameraHumanDetector, DepthScanner, GpsSensor, ImuSensor,
                      WifiCsiSensor)
from .types import Command
from .world import Drone, World


class SimLog:
    def __init__(self):
        self.events = []      # (t, kind, data)
        self.veto_events = []  # (t, reason)
        self.poses = []       # dicts
        self.csi_inferences = []  # every CSI inference seen, for audit tests
        self.cam_detections = 0  # every camera detection seen (incl. false positives)
        self.collisions = 0
        self.min_front = float("inf")
        self.front_samples = []
        # observability layer: reconstructable decisions with provenance
        self.decisions = []        # DecisionRecord.as_dict() entries
        self.health_transitions = []

    def decision(self, record):
        self.decisions.append(record.as_dict())

    def event(self, t, kind, data=None):
        self.events.append((t, kind, data))

    def veto(self, t, reason):
        self.veto_events.append((t, reason))
        self.event(t, "veto", {"reason": reason})

    def pose(self, t, drone, pose, state):
        self.poses.append({
            "t": round(t, 2), "x": round(drone.x, 3), "y": round(drone.y, 3),
            "z": round(drone.z, 2), "battery": round(drone.battery, 2),
            "sigma": round(pose.sigma, 3) if pose else None,
            "state": state})

    def csi(self, t, inference):
        self.csi_inferences.append({
            "t": round(t, 2), "present": inference.present,
            "p_present": round(inference.p_present, 4),
            "confidence": round(inference.confidence, 4),
            "uncertainty": round(inference.uncertainty, 4),
            "inferred": inference.inferred, "confirmed": inference.confirmed})


class Simulation:
    def __init__(self, world=None, goal=(52.0, 52.0), cfg=None, setup=None,
                 start=(5.0, 5.0), battery=100.0, injector=None):
        self.cfg = cfg or SimConfig()
        self.setup = setup or SensorSetup()
        master = random.Random(self.setup.seed)
        self.world = world or World(self.cfg.world_size, self.cfg.grid_res)
        self.drone = Drone(start[0], start[1], battery, self.cfg.world_size)
        self.log = SimLog()

        self.gps = GpsSensor(self.setup.gps_mode, self.cfg.gps_sigma,
                             self.setup.gps_degrade_factor,
                             self.setup.gps_jump_magnitude,
                             rng=random.Random(master.randint(0, 2**30 - 1)))
        self.imu = ImuSensor(self.setup.imu_mode, self.cfg.imu_sigma,
                             rng=random.Random(master.randint(0, 2**30 - 1)))
        self.depth = DepthScanner(self.setup.depth_mode,
                                  rng=random.Random(master.randint(0, 2**30 - 1)))
        self.cam = CameraHumanDetector(self.setup.cam_mode, self.setup.cam_p_detect,
                                       self.setup.cam_p_false,
                                       rng=random.Random(master.randint(0, 2**30 - 1)))
        self.csi = WifiCsiSensor(self.setup.csi_mode,
                                 rng=random.Random(master.randint(0, 2**30 - 1)))
        self.comms = CommsLink(self.setup.comms_mode, self.setup.comms_loss_at)

        self.localizer = Localizer(self.cfg, start[0], start[1])
        self.mapper = OccupancyMapper(self.cfg.world_size, self.cfg.grid_res)
        self.mission = Mission(goal, (start[0], start[1]), self.cfg)
        self.safety = SafetyLayer(self.cfg, (start[0], start[1]), self.cfg.world_size)
        self.avoid = AvoidanceLayer(self.cfg)
        self.health_monitor = HealthMonitor(self.cfg)
        self.injector = injector  # optional chaos harness between sensors and fusion
        self.t = 0.0
        self._prev_conflict = False
        # plausibility tracking for stuck depth sensor (frozen values, fresh t)
        self._ref_rays = None
        self._ref_pos = (start[0], start[1])

    def step(self):
        t, dt, cfg = self.t, self.cfg.dt, self.cfg

        # 1. sense
        gps_obs = self.gps.read(t, self.drone)
        imu_obs = self.imu.read(t, self.drone)
        depth_obs = self.depth.read(t, self.drone, self.world)
        cam_obs = self.cam.read(t, self.drone, self.world)
        csi_obs = self.csi.read(t, self.drone, self.world)
        # Stuck-sensor plausibility check: if the drone has clearly moved but
        # the depth scan is bit-identical, the sensor is frozen -> distrust it.
        depth_trusted = depth_obs is not None
        if depth_obs is not None:
            import math as _m
            if self._ref_rays is not None and depth_obs.rays == self._ref_rays:
                # scan identical to the reference: only plausible if we
                # haven't moved much since; large motion + identical scan
                # means the sensor is frozen
                moved = _m.hypot(self.drone.x - self._ref_pos[0],
                                 self.drone.y - self._ref_pos[1])
                if moved > 1.0:
                    depth_trusted = False
            else:
                self._ref_rays = depth_obs.rays
                self._ref_pos = (self.drone.x, self.drone.y)
        depth_age = (t - depth_obs.t) if depth_trusted else float("inf")

        # 1b. chaos layer may mangle the observation stream (Layer 4)
        obs_map = {"gps": gps_obs, "imu": imu_obs, "depth": depth_obs,
                   "camera": cam_obs, "wifi_csi": csi_obs}
        if self.injector is not None:
            obs_map = self.injector.intercept(t, obs_map)

        # 1c. contract validation at the boundary (Layer 1): a violating
        # observation is dropped and recorded, never "mostly used"
        verdicts = validate_all(obs_map)
        for name, v in verdicts.items():
            if not v.ok and v.reason != "missing":
                self.log.event(t, "contract_rejection",
                               {"sensor": name, "reason": v.reason})
                self.log.decision(record_contract_rejection(t, name, v.reason))
                obs_map[name] = None
        gps_obs, imu_obs = obs_map["gps"], obs_map["imu"]
        depth_obs, cam_obs = obs_map["depth"], obs_map["camera"]
        csi_obs = obs_map["wifi_csi"]
        if depth_obs is None or not depth_trusted:
            depth_age = float("inf")

        # 2. fuse + localize
        self.localizer.step(t, gps_obs, imu_obs)
        pose = self.localizer.estimate()

        # 2b. graded health states (Layer 2) + conflict provenance (Layer 3)
        ages = {name: ((t - obs.t) if obs is not None else float("inf"))
                for name, obs in obs_map.items()}
        flags = {"gps": self.localizer.health.conflict_active,
                 "depth": (depth_obs is not None and not depth_trusted)}
        for tr in self.health_monitor.step(t, verdicts, ages, flags):
            self.log.event(t, "health_transition", tr)
            self.log.health_transitions.append(tr)
            self.log.decision(record_health_transition(tr))
        if self.localizer.health.conflict_active and not self._prev_conflict:
            self.log.event(t, "conflict_declared", {"source": "gps"})
            self.log.decision(record_conflict(
                t, "gps", "innovation gate rejected fixes; source excluded"))
        self._prev_conflict = self.localizer.health.conflict_active

        # 3. map (world model, static geometry)
        self.mapper.integrate(self.drone, depth_obs, self.depth.max_range)

        # 4. perceive (probabilistic presence, never confirmed)
        presence = assess_presence(cam_obs, csi_obs)
        if csi_obs is not None:
            self.log.csi(t, csi_obs.inference)
        if cam_obs is not None:
            self.log.cam_detections += len(cam_obs.detections)

        # 5. mission proposes
        proposed = self.mission.step(t, self.drone, pose, self.mapper,
                                     presence, self.log)

        # 6. safety layer disposes (final authority)
        effective, veto_reason = self.safety.filter(
            proposed, t, self.drone, pose, self.localizer.health,
            depth_age,
            self.comms.lost_for(t), self.log)
        if veto_reason is not None:
            self.mission.on_override(effective, t, self.log)
            self.log.decision(record_veto(
                t, proposed.kind, effective.kind, veto_reason,
                pose.sigma if pose else None, "flight_control"))

        # 7. avoidance reflex scales motion down (cannot create new motion)
        effective = self.avoid.adjust(effective, depth_obs, t)

        # 8. flight control interface
        self.drone.apply(effective, dt, cfg)

        # 9. world evolves
        self.world.step(dt)

        # 10. bookkeeping
        if self.world.grid.occupied_xy(self.drone.x, self.drone.y):
            self.log.collisions += 1
            self.log.event(t, "collision", {})
        if depth_obs is not None:
            front = min(d for _, d in depth_obs.rays)
            self.log.min_front = min(self.log.min_front, front)
            self.log.front_samples.append(front)
        self.log.pose(t, self.drone, pose, self.mission.state)
        self.t += dt

    def run(self, duration):
        for _ in range(int(duration / self.cfg.dt)):
            if self.mission.state == Mission.DONE:
                break
            self.step()
        return self.log

    def veto_reasons(self):
        return [r for _, r in self.log.veto_events]

    def events_of(self, kind):
        return [e for e in self.log.events if e[1] == kind]
