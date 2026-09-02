"""HAL interfaces for vehicle and sensors."""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Tuple, Optional


class VehicleInterface(ABC):
    @abstractmethod
    def apply_thrust(self, thrust_accel: float) -> None:
        ...

    @abstractmethod
    def command_velocity(self, vx: float, vy: float) -> None:
        ...

    @abstractmethod
    def step(self, dt: float, environment: object | None = None, sim_time: float = 0.0) -> None:
        ...

    @abstractmethod
    def get_position(self) -> Tuple[float, float, float]:
        ...

    @abstractmethod
    def get_altitude(self) -> float:
        ...

    @abstractmethod
    def takeoff(self, *args, **kwargs) -> None:
        ...

    @abstractmethod
    def land(self, *args, **kwargs) -> None:
        ...


class GPSInterface(ABC):
    @abstractmethod
    def last_position(self) -> Optional[Tuple[float, float, float]]:
        ...

    @abstractmethod
    def is_healthy(self) -> bool:
        ...


class IMUInterface(ABC):
    @abstractmethod
    def last_accel(self) -> Tuple[float, float, float]:
        ...


class BarometerInterface(ABC):
    @abstractmethod
    def altitude(self) -> float:
        ...


class BatteryInterface(ABC):
    @abstractmethod
    def battery_level(self) -> float:
        ...


class NavigationInterface(ABC):
    @abstractmethod
    def active_waypoint(self):
        ...

    @abstractmethod
    def update_position(self, lat: float, lon: float, alt: float) -> str:
        ...
