"""2.5-D world model: occupancy grid, raycasting, humans, and drone body.

The world is deliberately simple: a square planar grid with axis-aligned
walls, used to exercise navigation, avoidance, and mapping logic. Humans are
sensing targets, NOT obstacles (documented limitation).
"""
import math
import random


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class Grid:
    def __init__(self, size=60.0, res=0.5):
        self.size = size
        self.res = res
        self.n = int(size / res)
        self.occ = set()  # occupied cells as (i, j)

    def cell(self, x, y):
        return (int(x / self.res), int(y / self.res))

    def in_bounds(self, i, j):
        return 0 <= i < self.n and 0 <= j < self.n

    def occupied_xy(self, x, y):
        i, j = self.cell(x, y)
        return (i, j) in self.occ

    def add_rect_wall(self, x0, y0, x1, y1):
        for i in range(int(x0 / self.res), int(x1 / self.res) + 1):
            for j in range(int(y0 / self.res), int(y1 / self.res) + 1):
                if self.in_bounds(i, j):
                    self.occ.add((i, j))

    def raycast(self, x, y, theta, max_range, step=0.15):
        """Distance to first occupied cell along the ray, or max_range/boundary."""
        c, s = math.cos(theta), math.sin(theta)
        d = 0.0
        while d < max_range:
            d += step
            px, py = x + c * d, y + s * d
            if not (0.0 <= px < self.size and 0.0 <= py < self.size):
                return max_range  # world edge treated as unobstructed
            if self.occupied_xy(px, py):
                return d
        return max_range


class Human:
    """A person in the world. `moving` people walk (producing Doppler-like
    CSI signatures); static people are much harder to detect via CSI."""

    def __init__(self, x, y, moving=False, seed=0):
        self.x, self.y = x, y
        self.moving = moving
        self._rng = random.Random(seed)
        self.heading = self._rng.uniform(0, 2 * math.pi)

    def step(self, dt, world, speed=0.7):
        if not self.moving:
            return
        if self._rng.random() < 0.05:
            self.heading += self._rng.uniform(-1.0, 1.0)
        nx = self.x + math.cos(self.heading) * speed * dt
        ny = self.y + math.sin(self.heading) * speed * dt
        if (1.0 < nx < world.size - 1.0 and 1.0 < ny < world.size - 1.0
                and not world.grid.occupied_xy(nx, ny)):
            self.x, self.y = nx, ny
        else:
            self.heading += math.pi / 2


class World:
    def __init__(self, size=60.0, res=0.5):
        self.size = size
        self.grid = Grid(size, res)
        self.humans = []

    def add_wall(self, x0, y0, x1, y1):
        self.grid.add_rect_wall(x0, y0, x1, y1)

    def add_human(self, x, y, moving=False, seed=0):
        h = Human(x, y, moving, seed)
        self.humans.append(h)
        return h

    def step(self, dt):
        for h in self.humans:
            h.step(dt, self)

    def los_clear(self, x0, y0, x1, y1):
        """True if the segment (x0,y0)->(x1,y1) crosses no occupied cell."""
        d = math.hypot(x1 - x0, y1 - y0)
        steps = max(1, int(d / 0.15))
        for k in range(1, steps + 1):
            px = x0 + (x1 - x0) * k / steps
            py = y0 + (y1 - y0) * k / steps
            if self.grid.occupied_xy(px, py):
                return False
        return True


class Drone:
    """Simple kinematic drone body with altitude abstraction and battery model."""

    def __init__(self, x=5.0, y=5.0, battery=100.0, size=60.0):
        self.x, self.y = x, y
        self.z = 0.0
        self.vx = self.vy = 0.0
        self.ax = self.ay = 0.0
        self.yaw = 0.0
        self.battery = battery
        self.home = (x, y)
        self.motors_on = False
        self.landed = True
        self.size = size

    def apply(self, cmd, dt, cfg):
        """Flight-control interface: accel-limited velocity tracking."""
        if cmd.kind == "takeoff":
            self.motors_on = True
            self.landed = False
            self.z = min(cfg.flight_altitude, self.z + 2.0 * dt)
            self._track(0.0, 0.0, dt, cfg)
        elif cmd.kind == "land":
            self.z = max(0.0, self.z - 1.5 * dt)
            self._track(0.0, 0.0, dt, cfg)
            if self.z <= 0.0:
                self.z = 0.0
                self.motors_on = False
                self.landed = True
                self.vx = self.vy = 0.0
        elif cmd.kind in ("velocity", "rtl", "loiter", "none"):
            if self.landed:
                return  # no lateral motion while on the ground
            if cmd.kind in ("loiter", "none"):
                self._track(0.0, 0.0, dt, cfg)
            else:
                self._track(cmd.vx, cmd.vy, dt, cfg)
        if self.motors_on:
            drain = cfg.battery_drain_per_s
            if math.hypot(self.vx, self.vy) > 0.1:
                drain += cfg.battery_drain_moving
            self.battery = max(0.0, self.battery - drain * dt)
        eps = 0.01
        self.x = min(max(self.x, eps), self.size - eps)
        self.y = min(max(self.y, eps), self.size - eps)

    def _track(self, tvx, tvy, dt, cfg):
        ax = (tvx - self.vx) / dt
        ay = (tvy - self.vy) / dt
        n = math.hypot(ax, ay)
        if n > cfg.max_accel:
            ax *= cfg.max_accel / n
            ay *= cfg.max_accel / n
        self.vx += ax * dt
        self.vy += ay * dt
        self.x += self.vx * dt
        self.y += self.vy * dt
        if math.hypot(self.vx, self.vy) > 0.05:
            self.yaw = math.atan2(self.vy, self.vx)
        self.ax, self.ay = ax, ay
