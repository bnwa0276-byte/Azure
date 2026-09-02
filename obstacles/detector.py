"""Obstacle detection utilities."""
from __future__ import annotations

from typing import Iterable, List, Tuple
from dataclasses import dataclass
import math

from .representation import Obstacle


@dataclass
class CollisionPrediction:
    will_collide: bool
    time_to_collision: float
    obstacle: Obstacle | None
    collision_point: Tuple[float, float, float] | None


class ObstacleDetector:
    """Detect potential collisions between a predicted straight-line
    horizontal path and nearby obstacles.

    The detector is simplistic: it projects the drone's current horizontal
    velocity over a time horizon and checks for intersection with circular
    obstacle footprints, accounting for vertical overlap.
    """

    def __init__(self, obstacles: Iterable[Obstacle], drone_radius: float = 0.5, horizon: float = 5.0):
        self.obstacles: List[Obstacle] = list(obstacles)
        self.drone_radius = float(drone_radius)
        self.horizon = float(horizon)

    def predict_collision(self, position: Tuple[float, float, float], velocity: Tuple[float, float, float], dt_step: float = 0.1) -> CollisionPrediction:
        px, py, pz = position
        vx, vy, vz = velocity

        # sample along straight-line path up to horizon
        steps = max(1, int(math.ceil(self.horizon / dt_step)))
        for i in range(1, steps + 1):
            t = i * dt_step
            if t > self.horizon:
                break
            sx = px + vx * t
            sy = py + vy * t
            sz = pz + vz * t
            for obs in self.obstacles:
                # effective radius includes drone radius
                dx = sx - obs.x
                dy = sy - obs.y
                dist2 = dx * dx + dy * dy
                thresh = (obs.radius + self.drone_radius) ** 2
                if dist2 <= thresh and (obs.z_min <= sz <= obs.z_max):
                    return CollisionPrediction(True, t, obs, (sx, sy, sz))

        return CollisionPrediction(False, float("inf"), None, None)
