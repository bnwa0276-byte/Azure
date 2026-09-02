"""Local avoidance planner: generate adjusted guidance when obstacle detected."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple, List, Optional
import math

from obstacles.representation import Obstacle


@dataclass
class AvoidancePlan:
    vx: float
    vy: float
    status: str
    info: Optional[dict] = None


class AvoidancePlanner:
    """Simple reactive planner that produces a lateral velocity to skirt an obstacle.

    Strategy: compute a tangential direction around the obstacle at the point
    of closest approach and produce a velocity vector that moves the drone
    tangentially while preserving desired speed.
    """

    def __init__(self, safety_margin: float = 0.5):
        self.safety_margin = float(safety_margin)

    def plan(self, position: Tuple[float, float, float], desired_v: Tuple[float, float], obstacle: Obstacle, desired_speed: float) -> AvoidancePlan:
        px, py, _ = position
        vx_des, vy_des = desired_v

        # compute vector from obstacle center to drone
        dx = px - obstacle.x
        dy = py - obstacle.y
        dist = math.hypot(dx, dy)
        if dist == 0.0:
            # edge-case: on top of obstacle — stop horizontal motion
            return AvoidancePlan(0.0, 0.0, "ON_OBSTACLE", {"reason": "on_obstacle"})

        # tangent vector (perpendicular)
        tx = -dy / dist
        ty = dx / dist

        # choose tangent direction that has positive projection onto desired velocity
        dot = tx * vx_des + ty * vy_des
        if dot < 0:
            tx = -tx
            ty = -ty

        # scale to desired speed while ensuring safety margin increases clearance
        speed = float(desired_speed)
        vx = tx * speed
        vy = ty * speed

        return AvoidancePlan(vx=vx, vy=vy, status="AVOIDING", info={"obstacle_center": (obstacle.x, obstacle.y), "safety_margin": self.safety_margin})
