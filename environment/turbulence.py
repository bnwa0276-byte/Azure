"""Turbulence generator (lightweight RNG-based)."""
from dataclasses import dataclass
from typing import Tuple, Optional
import random


@dataclass
class TurbulenceGenerator:
    strength: float = 0.0
    seed: Optional[int] = None

    def __post_init__(self):
        self._rng = random.Random(self.seed)

    def sample(self) -> Tuple[float, float, float]:
        # returns small random perturbations (x,y,z)
        tx = self._rng.uniform(-1.0, 1.0) * self.strength
        ty = self._rng.uniform(-1.0, 1.0) * self.strength
        tz = self._rng.uniform(-1.0, 1.0) * (self.strength * 0.2)
        return (tx, ty, tz)
