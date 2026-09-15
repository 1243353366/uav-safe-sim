"""Mapping: incremental occupancy grid built from depth-scan raycasts.

Cells at a ray hit are marked occupied; traversed cells are implicitly free.
This is the drone's *world model* of static geometry - deliberately separate
from the human-presence model (perception.py), which is always probabilistic.
"""
import math

from .types import DepthObs


class OccupancyMapper:
    def __init__(self, size, res):
        self.size = size
        self.res = res
        self.occ = set()  # (i, j) cells known to be occupied
        self.n = int(size / res)
        self.cells_scanned = 0

    def integrate(self, drone, depth: DepthObs, max_range=12.0):
        if depth is None:
            return
        for ang, d in depth.rays:
            self.cells_scanned += 1
            # A hit clearly before max range marks an occupied cell.
            if d < max_range - 0.3:
                hx = drone.x + math.cos(ang) * d
                hy = drone.y + math.sin(ang) * d
                i = int(hx / self.res)
                j = int(hy / self.res)
                if 0 <= i < self.n and 0 <= j < self.n:
                    self.occ.add((i, j))

    def blocked_cells(self):
        """Occupied cells plus a 1-cell inflation ring, for planning."""
        blocked = set()
        for (i, j) in self.occ:
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    blocked.add((i + di, j + dj))
        return blocked
