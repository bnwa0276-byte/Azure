from __future__ import annotations

"""Compute summary statistics from flight recorder entries."""

from typing import Iterable, List


class Statistics:
    def __init__(self, entries: Iterable):
        self._entries = list(entries)

    def mission_completion(self) -> float:
        if not self._entries:
            return 0.0
        last = self._entries[-1].telemetry or {}
        return 1.0 if last.get("mission_status") == "MISSION_COMPLETE" else 0.0

    def estimator_confidence_over_time(self) -> List[float]:
        out: List[float] = []
        for e in self._entries:
            out.append(float(e.telemetry.get("estimated", {}).get("confidence", 0.0)))
        return out

    def obstacle_avoidance_events(self) -> int:
        cnt = 0
        for e in self._entries:
            if e.telemetry.get("guidance", {}).get("status") == "AVOIDING":
                cnt += 1
            for ev in (e.events or []):
                if "AVOID" in ev.get("event", "") or "OBSTACLE" in ev.get("event", ""):
                    cnt += 1
        return cnt

    def max_overshoot(self) -> float:
        max_ov = 0.0
        for e in self._entries:
            telemetry = e.telemetry or {}
            target = telemetry.get("target_altitude")
            if target is None:
                continue
            est_alt = telemetry.get("estimated", {}).get("position", (0.0, 0.0, telemetry.get("altitude", 0.0)))[2]
            over = est_alt - float(target)
            if over > max_ov:
                max_ov = over
        return float(max_ov)

    def average_altitude_error(self) -> float:
        vals: List[float] = []
        for e in self._entries:
            telemetry = e.telemetry or {}
            if "target_altitude" not in telemetry:
                continue
            target = float(telemetry.get("target_altitude"))
            alt = float(telemetry.get("altitude", telemetry.get("estimated", {}).get("position", (0, 0, 0))[2]))
            vals.append(abs(alt - target))
        return float(sum(vals) / len(vals)) if vals else 0.0
