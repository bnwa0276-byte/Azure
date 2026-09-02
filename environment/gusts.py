"""Gust event definitions and helpers."""
from dataclasses import dataclass
from typing import Tuple, List


@dataclass
class GustEvent:
    start: float
    duration: float
    strength: Tuple[float, float]


def total_gust_at_time(gusts: List[GustEvent], t: float) -> Tuple[float, float]:
    gx, gy = 0.0, 0.0
    for g in gusts:
        if g.start <= t <= (g.start + g.duration):
            gx += g.strength[0]
            gy += g.strength[1]
    return (gx, gy)
