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
            telemetry = e.telemetry or {}
            est = telemetry.get("estimated")
            conf = est.get("confidence", 0.0) if isinstance(est, dict) else 0.0
            out.append(float(conf if conf is not None else 0.0))
        return out

    def obstacle_avoidance_events(self) -> int:
        cnt = 0
        for e in self._entries:
            telemetry = e.telemetry or {}
            guidance = telemetry.get("guidance")
            if isinstance(guidance, dict) and guidance.get("status") == "AVOIDING":
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
            est = telemetry.get("estimated")
            est_alt = None
            if isinstance(est, dict):
                pos = est.get("position")
                if isinstance(pos, (list, tuple)) and len(pos) > 2 and pos[2] is not None:
                    est_alt = pos[2]
            if est_alt is None:
                alt = telemetry.get("altitude")
                est_alt = alt if alt is not None else 0.0
            over = float(est_alt) - float(target)
            if over > max_ov:
                max_ov = over
        return float(max_ov)

    def average_altitude_error(self) -> float:
        vals: List[float] = []
        for e in self._entries:
            telemetry = e.telemetry or {}
            target = telemetry.get("target_altitude")
            if target is None:
                continue
            target_f = float(target)
            alt_val = telemetry.get("altitude")
            if alt_val is None:
                estimated = telemetry.get("estimated")
                if isinstance(estimated, dict):
                    pos = estimated.get("position")
                    if isinstance(pos, (list, tuple)) and len(pos) > 2 and pos[2] is not None:
                        alt_val = pos[2]
            if alt_val is None:
                alt_val = 0.0
            alt = float(alt_val)
            vals.append(abs(alt - target_f))
        return float(sum(vals) / len(vals)) if vals else 0.0
