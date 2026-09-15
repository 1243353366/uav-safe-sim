"""Sensor fusion: 2-D EKF over [x, y, vx, vy] with chi-square innovation gating.

Design principles (see docs/03-sensor-fusion.md):
- IMU accelerations drive the prediction step.
- GPS fixes drive the update step, but each fix must pass an innovation gate
  (Mahalanobis distance, chi^2 99.9% / 2 dof). A fix that contradicts the
  dead-reckoning trajectory is rejected, not blindly fused.
- A streak of rejected fixes is declared a sensor conflict; the conflicting
  source is excluded and the system degrades toward its safe state.
- With no usable GPS fix, position uncertainty is grown conservatively each
  tick (noifix_pos_var_growth) - uncertainty must reflect what we do NOT know.
"""
import math

import numpy as np

from .types import PoseEstimate


class SensorHealth:
    """Health summary surfaced to the safety layer every tick."""

    def __init__(self):
        self.gps_age = float("inf")
        self.imu_age = float("inf")
        self.gps_usable = False
        self.conflict_active = False
        self.conflict_t0 = None  # time the conflict was declared


class EkfFusion:
    def __init__(self, x0, y0, cfg):
        self.cfg = cfg
        self.x = np.array([x0, y0, 0.0, 0.0], dtype=float)
        self.P = np.diag([1.0, 1.0, 1.0, 1.0])
        self.q = 0.6 ** 2  # accel process-noise variance (m/s^2)^2
        self.reject_streak = 0
        self.gps_excluded = False
        self.t_last = None
        self.health = SensorHealth()

    # ---- prediction from IMU ----
    def predict(self, t, imu):
        dt = self.cfg.dt if self.t_last is None else max(1e-6, t - self.t_last)
        self.t_last = t
        if imu is not None and (t - imu.t) <= self.cfg.imu_max_age:
            a = np.array([imu.ax, imu.ay])
            self.health.imu_age = t - imu.t
            q_scale = 1.0
        else:
            # No usable IMU: coast, but trust the prediction much less.
            a = np.zeros(2)
            q_scale = 6.0
            self.health.imu_age = float("inf")
        F = np.eye(4)
        F[0, 2] = dt
        F[1, 3] = dt
        # control matrix: world-frame accel (ax, ay) -> position and velocity
        G = np.array([[0.5 * dt * dt, 0.0],
                      [0.0, 0.5 * dt * dt],
                      [dt, 0.0],
                      [0.0, dt]])
        self.x = F @ self.x + G @ a
        self.P = F @ self.P @ F.T + (G @ G.T) * (self.q * q_scale)

    # ---- gated GPS update ----
    def update_gps(self, t, obs):
        if obs is None or (t - obs.t) > self.cfg.gps_max_age:
            self.health.gps_age = float("inf")
            self.health.gps_usable = False
            # Conservative growth while no absolute fix is available.
            self.P[0, 0] += self.cfg.nofix_pos_var_growth
            self.P[1, 1] += self.cfg.nofix_pos_var_growth
            return
        self.health.gps_age = t - obs.t
        self.health.gps_usable = True
        if self.gps_excluded:
            return  # conflict declared; do not re-ingest a contradicting source
        H = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])
        z = np.array([obs.x, obs.y])
        R = np.eye(2) * (obs.sigma ** 2)
        nu = z - H @ self.x
        S = H @ self.P @ H.T + R
        d2 = float(nu @ np.linalg.solve(S, nu))
        if d2 > self.cfg.chi2_gate:
            self.reject_streak += 1
            if self.reject_streak >= self.cfg.conflict_reject_streak:
                self.gps_excluded = True
                self.health.conflict_active = True
                if self.health.conflict_t0 is None:
                    self.health.conflict_t0 = t
            return
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ nu
        I_KH = np.eye(4) - K @ H
        self.P = I_KH @ self.P
        self.reject_streak = max(0, self.reject_streak - 1)

    def estimate(self) -> PoseEstimate:
        w = np.linalg.eigvalsh(self.P[:2, :2])
        sigma = math.sqrt(max(float(w.max()), 0.0))
        return PoseEstimate(x=float(self.x[0]), y=float(self.x[1]),
                           vx=float(self.x[2]), vy=float(self.x[3]),
                           sigma=sigma, conflict=self.health.conflict_active)
