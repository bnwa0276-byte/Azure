"""Navigation subsystem for the autonomous drone platform."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, radians, sqrt
from typing import Iterable, List, Optional


@dataclass
class Waypoint:
    """Represents a geographic location and target altitude."""

    name: str
    latitude: float
    longitude: float
    altitude: float


class Mission:
    """Stores an ordered list of waypoints and tracks progress."""

    def __init__(self, waypoints: Iterable[Waypoint]) -> None:
        self.waypoints: List[Waypoint] = list(waypoints)
        self.current_index = 0

    @property
    def active_waypoint(self) -> Optional[Waypoint]:
        """Return the currently active waypoint, or None if the mission is complete."""
        if self.current_index < len(self.waypoints):
            return self.waypoints[self.current_index]
        return None

    @property
    def is_complete(self) -> bool:
        """Return whether the mission has reached its final waypoint."""
        return self.active_waypoint is None

    def advance(self) -> None:
        """Advance to the next waypoint in the mission."""
        if not self.is_complete:
            self.current_index += 1

    def progress_report(self) -> str:
        """Return a simple progress summary for the mission."""
        return f"{self.current_index}/{len(self.waypoints)} waypoints completed"


class NavigationSystem:
    """Manages waypoint progression and distance calculations for a mission."""

    WAYPOINT_THRESHOLD_METERS = 5.0

    def __init__(self, mission: Mission) -> None:
        self.mission = mission
        self.current_distance = float("inf")

    @property
    def active_waypoint(self) -> Optional[Waypoint]:
        """Return the currently active waypoint."""
        return self.mission.active_waypoint

    def distance_to_active_waypoint(
        self,
        latitude: float,
        longitude: float,
        altitude: float,
    ) -> float:
        """Calculate a simple simulated distance to the active waypoint."""
        waypoint = self.active_waypoint
        if waypoint is None:
            return 0.0

        meters_per_degree = 111_000.0
        lat_diff = (waypoint.latitude - latitude) * meters_per_degree
        lon_diff = (waypoint.longitude - longitude) * meters_per_degree * cos(radians(latitude))
        alt_diff = waypoint.altitude - altitude
        self.current_distance = sqrt(lat_diff ** 2 + lon_diff ** 2 + alt_diff ** 2)
        return self.current_distance

    def update_position(
        self,
        latitude: float,
        longitude: float,
        altitude: float,
    ) -> str:
        """Update current position and advance the mission if the waypoint is reached."""
        distance = self.distance_to_active_waypoint(latitude, longitude, altitude)
        if self.active_waypoint is None:
            return "MISSION_COMPLETE"

        if distance <= self.WAYPOINT_THRESHOLD_METERS:
            self.mission.advance()
            if self.mission.is_complete:
                return "MISSION_COMPLETE"
            return "WAYPOINT_REACHED"

        return "EN_ROUTE"

    def mission_status(self) -> str:
        """Return a summary of the mission status."""
        if self.mission.is_complete:
            return "COMPLETE"
        return self.mission.progress_report()
