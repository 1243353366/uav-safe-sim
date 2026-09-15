"""Central configuration for the UAV research simulation.

All safety-relevant thresholds live here so they are auditable in one place.
Any change to safety behavior must be reflected in the failure-mode tests.
"""
from dataclasses import dataclass


@dataclass
class SimConfig:
    # ---- time / dynamics ----
    dt: float = 0.1                  # s
    max_speed: float = 4.0            # m/s
    max_accel: float = 3.0           # m/s^2
    max_decel: float = 3.0           # m/s^2 (used by avoidance stopping distance)
    flight_altitude: float = 3.0     # m (2.5-D abstraction: altitude is not navigated)

    # ---- battery (percent of capacity) ----
    battery_rth: float = 25.0        # at/below: return to home
    battery_land: float = 12.0       # at/below: land immediately wherever we are
    battery_drain_per_s: float = 0.08
    battery_drain_moving: float = 0.04

    # ---- sensor fusion / localization ----
    gps_sigma: float = 0.5           # nominal 1-sigma position noise (m)
    gps_max_age: float = 2.0         # s; older fixes are discarded as stale
    imu_max_age: float = 0.5         # s
    imu_sigma: float = 0.15          # m/s^2
    chi2_gate: float = 13.816        # chi^2(2 dof, 99.9%): innovation acceptance gate
    conflict_reject_streak: int = 5  # consecutive gated-out fixes => declared conflict
    nofix_pos_var_growth: float = 0.1  # var/s added while no usable GPS fix
    max_uncertainty: float = 6.0     # m sigma; beyond this localization is untrustworthy

    # ---- safety layer ----
    conflict_loiter_max: float = 30.0  # s; loiter on conflict, then land
    comms_loss_rtl_after: float = 10.0  # s of lost link before forced RTL
    depth_max_age: float = 1.0       # s; older depth data is unsafe to act on
    geofence_margin: float = 1.0     # m inside world boundary

    # ---- obstacle avoidance ----
    obstacle_stop_margin: float = 1.5  # m kept in front of obstacles
    avoid_cone: float = 0.61           # rad (~35 deg) half-angle of "forward" cone

    # ---- mission ----
    replan_period: float = 2.0
    waypoint_reach_radius: float = 0.8
    goal_reach_radius: float = 1.0
    home_reach_radius: float = 1.2
    inspect_duration: float = 8.0
    presence_confirm_threshold: float = 0.55  # fused confidence to trigger INSPECT

    # ---- world ----
    world_size: float = 60.0         # m (square world)
    grid_res: float = 0.5            # m/cell


@dataclass
class SensorSetup:
    """Failure-injection knobs. Modes simulate the scenarios required by the
    research protocol: noise, missing data, and contradictory observations."""
    gps_mode: str = "ok"            # ok | degraded | drop | stale | bias_jump
    imu_mode: str = "ok"            # ok | drop
    depth_mode: str = "ok"          # ok | drop | stuck
    cam_mode: str = "ok"            # ok | drop
    cam_p_detect: float = 0.9
    cam_p_false: float = 0.02
    csi_mode: str = "ok"            # ok | drop
    comms_mode: str = "ok"          # ok | lost
    comms_loss_at: float = 15.0     # s, when comms_mode == "lost"
    gps_degrade_factor: float = 10.0  # true sigma multiplier (reported sigma stays nominal)
    gps_jump_magnitude: tuple = (40.0, 25.0)
    seed: int = 7
