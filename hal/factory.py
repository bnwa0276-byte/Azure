"""Factory to create HAL implementations based on configuration."""
from __future__ import annotations

from typing import Literal, Optional
from hal.simulated import SimulatedVehicle, SimulatedSensors, SimulatedNavigation


def create_vehicle(kind: Literal["simulated"] = "simulated", **kwargs) -> SimulatedVehicle:
    if kind == "simulated":
        return SimulatedVehicle(kwargs.get("drone", None))
    # placeholder for future hardware backends
    raise ValueError(f"Unknown vehicle kind: {kind}")

def create_sensors(kind: Literal["simulated"] = "simulated", health=None):
    if kind == "simulated":
        return SimulatedSensors(health)
    raise ValueError(f"Unknown sensors kind: {kind}")

def create_navigation(kind: Literal["simulated"] = "simulated", navigation=None):
    if kind == "simulated":
        return SimulatedNavigation(navigation)
    raise ValueError(f"Unknown navigation kind: {kind}")
