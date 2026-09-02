"""Environment composed from modular subcomponents."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Tuple, List, Optional

from .steady import SteadyWind
from .gusts import GustEvent, total_gust_at_time
from .turbulence import TurbulenceGenerator


@dataclass
class Environment:
    steady_wind: Tuple[float, float] = (0.0, 0.0)
    gusts: List[GustEvent] = field(default_factory=list)
    turbulence_strength: float = 0.0
    enabled: bool = True
    drag_coef: float = 0.5
    seed: Optional[int] = None

    def __post_init__(self):
        self.steady = SteadyWind(*self.steady_wind)
        self.turbulence = TurbulenceGenerator(strength=self.turbulence_strength, seed=self.seed)
        self._rng = self.turbulence._rng

    def get_wind_velocity(self, position: Tuple[float, float, float], velocity: Tuple[float, float, float], t: float) -> Tuple[float, float]:
        """Return wind velocity vector (wx, wy) at given time and position."""
        if not self.enabled:
            return (0.0, 0.0)
        sx, sy = self.steady.as_tuple()
        gx, gy = total_gust_at_time(self.gusts, t)
        tx, ty, _ = self.turbulence.sample()
        return (sx + gx + tx, sy + gy + ty)

    def get_external_acceleration(self, position: Tuple[float, float, float], velocity: Tuple[float, float, float], t: float) -> Tuple[float, float, float]:
        """Return external acceleration (ax, ay, az) in m/s^2 applied to vehicle."""
        if not self.enabled:
            return (0.0, 0.0, 0.0)

        wx, wy = self.get_wind_velocity(position, velocity, t)
        vx, vy, vz = velocity
        rel_x = wx - vx
        rel_y = wy - vy

        ax = rel_x * self.drag_coef
        ay = rel_y * self.drag_coef

        # small vertical turbulence
        _, _, tz = self.turbulence.sample()
        az = tz
        return (ax, ay, az)

    def get_barometer_noise(self) -> float:
        if not self.enabled:
            return 0.0
        return self._rng.gauss(0.0, 0.02 * max(1.0, self.turbulence_strength))

    def get_gps_noise(self) -> Tuple[float, float, float]:
        if not self.enabled:
            return (0.0, 0.0, 0.0)
        return (
            self._rng.gauss(0.0, 0.5),
            self._rng.gauss(0.0, 0.5),
            self._rng.gauss(0.0, 0.2),
        )
