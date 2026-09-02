"""Simulated motor sensor for the autonomous drone."""

from __future__ import annotations

import logging
from typing import Any

from .base import Sensor, SensorStatus

logger = logging.getLogger(__name__)


class MotorSensor(Sensor):
    """Simulates motor status reporting."""

    def __init__(self, status: SensorStatus = SensorStatus.OK) -> None:
        self.name = "Motors"
        self.status = SensorStatus.OK
        self.initialize()
        self.update(status=status)

    def initialize(self) -> None:
        logger.info("Initializing motor sensor.")
        self.status = SensorStatus.OK

    def update(self, *args: Any, **kwargs: Any) -> None:
        status = kwargs.get("status", args[0] if args else SensorStatus.OK)
        self.status = status
        logger.info("Motor sensor status updated to %s", self.status.value)

    def is_healthy(self) -> bool:
        return self.status == SensorStatus.OK

    def health_report(self) -> str:
        return f"{self.name}={self.status.value}"
