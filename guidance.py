"""Guidance system: computes desired horizontal velocity toward active waypoint.

Independent of PhysicsEngine and FlightController; consumes `NavigationSystem`.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import cos, radians, sqrt
from typing import Optional, Iterable

from navigation import NavigationSystem
from obstacles.detector import ObstacleDetector, CollisionPrediction
from obstacles.planner import AvoidancePlanner, AvoidancePlan


@dataclass
class GuidanceCommand:
    vx: float = 0.0
    vy: float = 0.0
    desired_speed: float = 0.0
    status: str = "EN_ROUTE"
    distance_m: float = 0.0


class GuidanceSystem:
    """Compute horizontal guidance commands toward the active waypoint.

    GuidanceSystem uses simple equirectangular projection to convert
    waypoint lat/lon differences into local-meter offsets and computes a
    desired horizontal velocity vector limited by `desired_speed`.
    """

    def __init__(self, navigation: NavigationSystem, desired_speed: float = 1.0, acceptance_radius_m: Optional[float] = None):
        self.navigation = navigation
        self.desired_speed = float(desired_speed)
        # if not set, use navigation threshold
        self.acceptance_radius = float(acceptance_radius_m) if acceptance_radius_m is not None else float(navigation.WAYPOINT_THRESHOLD_METERS)
        # obstacle-aware components
        self.obstacle_detector: ObstacleDetector | None = None
        self.avoidance_planner: AvoidancePlanner | None = None

    def set_obstacles(self, obstacles: Iterable[object]) -> None:
        try:
            self.obstacle_detector = ObstacleDetector(obstacles)
            self.avoidance_planner = AvoidancePlanner()
        except Exception:
            self.obstacle_detector = None
            self.avoidance_planner = None

    def compute_command(self, latitude: float, longitude: float, altitude: float) -> GuidanceCommand:
        """Compute a guidance command for the current active waypoint.

        Returns a `GuidanceCommand` containing desired horizontal velocity (vx, vy)
        in meters per second, a status string, and the current distance to the
        waypoint in meters.
        """
        wp = self.navigation.active_waypoint
        if wp is None:
            return GuidanceCommand(vx=0.0, vy=0.0, desired_speed=0.0, status="MISSION_COMPLETE", distance_m=0.0)

        meters_per_degree = 111_000.0
        lat_diff = (wp.latitude - latitude) * meters_per_degree
        lon_diff = (wp.longitude - longitude) * meters_per_degree * cos(radians(latitude))
        dist = sqrt(lat_diff ** 2 + lon_diff ** 2)

        # If within acceptance radius, advance mission via navigation subsystem
        if dist <= self.acceptance_radius:
            result = self.navigation.update_position(latitude, longitude, altitude)
            return GuidanceCommand(vx=0.0, vy=0.0, desired_speed=0.0, status=result, distance_m=dist)

        # compute normalized direction and scale by desired_speed
        if dist == 0.0:
            base_vx = base_vy = 0.0
        else:
            base_vx = (lat_diff / dist) * self.desired_speed
            base_vy = (lon_diff / dist) * self.desired_speed

        # obstacle detection & avoidance
        if self.obstacle_detector is not None and self.avoidance_planner is not None:
            # translate lat/lon diffs to local meters to construct a position/velocity
            # assume drone at (0,0) local frame when computing guidance
            pred = self.obstacle_detector.predict_collision((0.0, 0.0, altitude), (base_vx, base_vy, 0.0))
            if pred.will_collide and pred.obstacle is not None:
                plan = self.avoidance_planner.plan((0.0, 0.0, altitude), (base_vx, base_vy), pred.obstacle, self.desired_speed)
                return GuidanceCommand(vx=plan.vx, vy=plan.vy, desired_speed=self.desired_speed, status=plan.status, distance_m=dist)

        return GuidanceCommand(vx=base_vx, vy=base_vy, desired_speed=self.desired_speed, status="EN_ROUTE", distance_m=dist)
