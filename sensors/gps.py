"""Simulated GPS sensor for the autonomous drone."""

from __future__ import annotations

import logging
from typing import Any

from .base import Sensor, SensorStatus

logger = logging.getLogger(__name__)


class GPSSensor(Sensor):
    """Simulates GPS availability state."""

    def __init__(self, available: bool = True) -> None:
        self.name = "GPS"
        self.available = False
        self.status = SensorStatus.FAILED
        self.initialize()
        self.update(available=available)

    def initialize(self) -> None:
        logger.info("Initializing GPS sensor.")
        self.status = SensorStatus.FAILED

    def update(self, *args: Any, **kwargs: Any) -> None:
        available = bool(kwargs.get("available", args[0] if args else False))
        self.available = available
        self.status = SensorStatus.OK if self.available else SensorStatus.FAILED
        logger.info("GPS sensor availability updated to %s", self.available)

    def is_healthy(self) -> bool:
        return self.status == SensorStatus.OK

    def health_report(self) -> str:
        return f"{self.name}={self.available}/{self.status.value}"

    def read_from_physics(self, physics: object, environment: object | None = None) -> None:
        """Read a noisy position from the physics object if available."""
        try:
            pos = tuple(getattr(physics, "position"))
        except Exception:
            pos = (0.0, 0.0, 0.0)

        # add optional GPS noise
        nx = ny = nz = 0.0
        if environment is not None and hasattr(environment, "get_gps_noise"):
            try:
                nx, ny, nz = environment.get_gps_noise()
            except Exception:
                nx = ny = nz = 0.0

        # store last known noisy position for external users
        self.last_position = (pos[0] + nx, pos[1] + ny, pos[2] + nz)
