"""Steady wind model component."""
from dataclasses import dataclass
from typing import Tuple


@dataclass
class SteadyWind:
    x: float = 0.0
    y: float = 0.0

    def as_tuple(self) -> Tuple[float, float]:
        return (float(self.x), float(self.y))
