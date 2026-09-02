from __future__ import annotations

"""Lightweight charting helpers. Returns simple figure-like objects (dicts) for headless environments."""

from typing import Iterable, Tuple, Any, List


def altitude_chart(series: Iterable[Tuple[float, float]]) -> Any:
    # series: list of (time, altitude)
    return {"type": "altitude", "series": list(series)}


def confidence_chart(series: Iterable[float]) -> Any:
    return {"type": "confidence", "series": list(series)}
