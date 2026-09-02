"""Simulated implementations of HAL interfaces that adapt existing simulator classes."""
from __future__ import annotations

from typing import Optional, Tuple, Iterable
from hal.interfaces import VehicleInterface, GPSInterface, IMUInterface, BarometerInterface, BatteryInterface, NavigationInterface
from drone import Drone
from navigation import NavigationSystem


class SimulatedVehicle(VehicleInterface):
    def __init__(self, drone: Optional[Drone] = None):
        self._drone = drone if drone is not None else Drone()

    def apply_thrust(self, thrust_accel: float) -> None:
        self._drone.apply_thrust(thrust_accel)

    def command_velocity(self, vx: float, vy: float) -> None:
        self._drone.command_velocity(vx, vy)

    def step(self, dt: float, environment: object | None = None, sim_time: float = 0.0) -> None:
        self._drone.step_physics(dt, environment=environment, sim_time=sim_time)

    def get_position(self) -> Tuple[float, float, float]:
        return tuple(self._drone.physics.position)

    def get_altitude(self) -> float:
        return float(self._drone.altitude)

    def takeoff(self, *args, **kwargs) -> None:
        return self._drone.takeoff(*args, **kwargs)

    def land(self, *args, **kwargs) -> None:
        return self._drone.land(*args, **kwargs)


class SimulatedSensors(GPSInterface, IMUInterface, BarometerInterface, BatteryInterface):
    def __init__(self, health: object):
        # health is expected to be HealthStatus from `drone.py`
        self._health = health

    def last_position(self) -> Optional[Tuple[float, float, float]]:
        try:
            return tuple(getattr(self._health.gps, "last_position"))
        except Exception:
            return None

    def is_healthy(self) -> bool:
        return self._health.gps.is_healthy()

    def last_accel(self) -> Tuple[float, float, float]:
        try:
            return tuple(getattr(self._health.imu, "last_accel"))
        except Exception:
            return (0.0, 0.0, 0.0)

    def altitude(self) -> float:
        try:
            return float(getattr(self._health.barometer, "altitude", 0.0))
        except Exception:
            return 0.0

    def battery_level(self) -> float:
        try:
            return float(getattr(self._health.battery, "battery_level", 0.0))
        except Exception:
            return 0.0


class SimulatedNavigation(NavigationInterface):
    def __init__(self, navigation: Optional[NavigationSystem] = None):
        self._nav = navigation if navigation is not None else NavigationSystem([])

    def active_waypoint(self):
        return self._nav.active_waypoint

    def update_position(self, lat: float, lon: float, alt: float) -> str:
        return self._nav.update_position(lat, lon, alt)
