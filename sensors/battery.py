"""Simulated battery sensor for the autonomous drone."""

from __future__ import annotations

import logging
from typing import Any

from .base import Sensor, SensorStatus

logger = logging.getLogger(__name__)


class BatterySensor(Sensor):
    """Simulates a battery sensor with health thresholds."""

    MIN_ARMING_LEVEL = 30.0
    CRITICAL_LEVEL = 10.0

    def __init__(self, battery_level: float = 100.0) -> None:
        self.name = "Battery"
        self.battery_level = 0.0
        self.status = SensorStatus.OK
        self.initialize()
        self.update(battery_level)

    def initialize(self) -> None:
        logger.info("Initializing battery sensor.")
        self.status = SensorStatus.OK

    def update(self, *args: Any, **kwargs: Any) -> None:
        battery_level = float(kwargs.get("battery_level", args[0] if args else 0.0))
        self.battery_level = max(0.0, min(100.0, battery_level))
        self._refresh_status()
        logger.info("Battery sensor updated to %.1f%% (%s)", self.battery_level, self.status.value)

    def _refresh_status(self) -> None:
        if self.battery_level <= self.CRITICAL_LEVEL:
            self.status = SensorStatus.FAILED
        elif self.battery_level < self.MIN_ARMING_LEVEL:
            self.status = SensorStatus.DEGRADED
        else:
            self.status = SensorStatus.OK

    def is_healthy(self) -> bool:
        return self.status == SensorStatus.OK

    def is_critical(self) -> bool:
        return self.status == SensorStatus.FAILED

    def health_report(self) -> str:
        return f"{self.name}={self.battery_level:.1f}%/{self.status.value}"
