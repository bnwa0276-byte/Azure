"""Environment configuration and flags."""
from dataclasses import dataclass
from typing import Tuple, List, Optional


@dataclass
class EnvironmentConfig:
    steady_wind: Tuple[float, float] = (0.0, 0.0)
    gusts: List[tuple] = None
    turbulence_strength: float = 0.0
    enabled: bool = True
    drag_coef: float = 0.5
    seed: Optional[int] = None

    def __post_init__(self):
        if self.gusts is None:
            self.gusts = []
