"""Base sensor abstractions for autonomous drone sensors."""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from enum import Enum
from typing import Any

logger = logging.getLogger(__name__)


class SensorStatus(str, Enum):
    """Enumerates generic sensor health statuses."""

    OK = "OK"
    DEGRADED = "DEGRADED"
    FAILED = "FAILED"


class Sensor(ABC):
    """Abstract base class for all drone sensors."""

    name: str
    status: SensorStatus

    @abstractmethod
    def initialize(self) -> None:
        """Initialize sensor hardware or simulation state."""

    @abstractmethod
    def update(self, *args: Any, **kwargs: Any) -> None:
        """Update the sensor reading or internal state."""

    @abstractmethod
    def is_healthy(self) -> bool:
        """Return whether the sensor is healthy."""

    def health_report(self) -> str:
        """Return a human-readable sensor health summary."""
        return f"{self.name}={self.status.value}"
