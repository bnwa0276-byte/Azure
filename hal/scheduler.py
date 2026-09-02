from __future__ import annotations

"""Simple deterministic scheduler for simulation time.

Tasks registered with a frequency (Hz) will be invoked when their scheduled
time is reached or passed by the simulated clock. Designed to be ticked with
monotonic simulation time from the simulation runner.
"""

from typing import Callable, List, Tuple


class Scheduler:
    def __init__(self):
        # tasks: list of (callback, period, next_time)
        self._tasks: List[Tuple[Callable[[float], None], float, float]] = []

    def schedule(self, callback: Callable[[float], None], rate_hz: float, start_time: float = 0.0) -> None:
        period = 1.0 / float(rate_hz) if rate_hz > 0 else float("inf")
        self._tasks.append((callback, period, start_time))

    def tick(self, now: float) -> None:
        updated: List[Tuple[Callable[[float], None], float, float]] = []
        for cb, period, next_time in self._tasks:
            if now + 1e-12 >= next_time:
                try:
                    cb(now)
                except Exception:
                    pass
                next_time = next_time + period
            updated.append((cb, period, next_time))
        self._tasks = updated
