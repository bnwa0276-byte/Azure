"""Simulated barometer sensor for the autonomous drone."""

from __future__ import annotations

import logging
from typing import Any

from .base import Sensor, SensorStatus

logger = logging.getLogger(__name__)


class BarometerSensor(Sensor):
    """Simulates barometer readings and health state."""

    def __init__(self, altitude: float = 0.0) -> None:
        self.name = "Barometer"
        self.altitude = 0.0
        self.status = SensorStatus.OK
        self.initialize()
        self.update(altitude=altitude)

    def initialize(self) -> None:
        logger.info("Initializing barometer sensor.")
        self.status = SensorStatus.OK

    def update(self, *args: Any, **kwargs: Any) -> None:
        self.altitude = float(kwargs.get("altitude", args[0] if args else 0.0))
        self.status = SensorStatus.OK
        logger.info("Barometer sensor altitude updated to %.1f", self.altitude)

    def read_from_physics(self, physics: object, environment: object | None = None) -> None:
        """Read altitude from a physics simulation object without modifying it.

        The physics object is expected to expose a `position` attribute
        (x, y, z). This method should only read simulation state and update
        the sensor's internal reading.
        """
        try:
            z = float(getattr(physics, "position")[2])
        except Exception:
            z = 0.0
        # add optional measurement noise from environment
        noise = 0.0
        if environment is not None and hasattr(environment, "get_barometer_noise"):
            try:
                noise = float(environment.get_barometer_noise())
            except Exception:
                noise = 0.0
        self.update(altitude=z + noise)

    def is_healthy(self) -> bool:
        return self.status == SensorStatus.OK

    def health_report(self) -> str:
        return f"{self.name}={self.altitude:.1f}m/{self.status.value}"
