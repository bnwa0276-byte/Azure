"""Obstacle representations."""
from dataclasses import dataclass
from typing import Tuple


@dataclass
class Obstacle:
    """A simple cylindrical obstacle described by a horizontal center (x,y),
    a radius (meters), and a vertical extent (z_min, z_max)."""
    x: float
    y: float
    radius: float
    z_min: float = 0.0
    z_max: float = 10.0

    def contains_point(self, px: float, py: float, pz: float) -> bool:
        dx = px - self.x
        dy = py - self.y
        inside_h = (dx * dx + dy * dy) <= (self.radius * self.radius)
        inside_v = (self.z_min <= pz <= self.z_max)
        return inside_h and inside_v
