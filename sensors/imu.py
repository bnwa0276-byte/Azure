"""Simulated IMU sensor for the autonomous drone."""

from __future__ import annotations

import logging
from typing import Any

from .base import Sensor, SensorStatus

logger = logging.getLogger(__name__)


class IMUSensor(Sensor):
    """Simulates IMU status reporting."""

    def __init__(self, status: SensorStatus = SensorStatus.OK) -> None:
        self.name = "IMU"
        self.status = SensorStatus.OK
        self.initialize()
        self.update(status=status)

    def initialize(self) -> None:
        logger.info("Initializing IMU sensor.")
        self.status = SensorStatus.OK

    def update(self, *args: Any, **kwargs: Any) -> None:
        status = kwargs.get("status", args[0] if args else SensorStatus.OK)
        self.status = status
        logger.info("IMU sensor status updated to %s", self.status.value)

    def is_healthy(self) -> bool:
        return self.status == SensorStatus.OK

    def health_report(self) -> str:
        return f"{self.name}={self.status.value}"

    def read_from_physics(self, physics: object, environment: object | None = None) -> None:
        """Read acceleration from the physics object if available and store it.

        This method provides a simple IMU acceleration reading derived from the
        physics engine's reported acceleration. It does not modify simulation
        state.
        """
        try:
            acc = getattr(physics, "acceleration", (0.0, 0.0, 0.0))
            # store last known acceleration for fusion
            self.last_accel = tuple(float(a) for a in acc)
        except Exception:
            self.last_accel = (0.0, 0.0, 0.0)
