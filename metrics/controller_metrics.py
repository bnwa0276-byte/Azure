"""Simple controller performance metrics collector."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple, Optional
import math


@dataclass
class ControllerMetrics:
    samples: List[Tuple[float, float, float]] = field(default_factory=list)
    # samples are tuples (time, altitude, target)

    def sample(self, t: float, altitude: float, target: Optional[float]) -> None:
        self.samples.append((float(t), float(altitude), float(target) if target is not None else float('nan')))

    def max_overshoot(self) -> float:
        if not self.samples:
            return 0.0
        overs = 0.0
        for _, alt, targ in self.samples:
            if math.isfinite(targ) and alt > targ:
                overs = max(overs, alt - targ)
        return overs

    def average_abs_error(self) -> float:
        if not self.samples:
            return 0.0
        errs = [abs(alt - targ) for _, alt, targ in self.samples if math.isfinite(targ)]
        return sum(errs) / len(errs) if errs else 0.0

    def settling_time(self, tolerance: float = 0.5, settle_window: float = 0.5) -> Optional[float]:
        # find first time after which error remains within tolerance for settle_window seconds
        if not self.samples:
            return None
        # create time-indexed errors
        times = [s[0] for s in self.samples]
        errs = [abs(s[1] - s[2]) for s in self.samples]
        n = len(times)
        for i in range(n):
            if errs[i] <= tolerance:
                # check window
                t0 = times[i]
                # ensure we have samples up to t0 + settle_window
                j = i
                while j < n and times[j] - t0 <= settle_window:
                    if errs[j] > tolerance:
                        break
                    j += 1
                if j == n or times[j-1] - t0 >= settle_window:
                    return t0
        return None

    def recovery_time_after(self, disturbance_time: float, tolerance: float = 0.5) -> Optional[float]:
        # time from disturbance_time to first time when error <= tolerance
        if not self.samples:
            return None
        for t, alt, targ in self.samples:
            if t >= disturbance_time and abs(alt - targ) <= tolerance:
                return t - disturbance_time
        return None
