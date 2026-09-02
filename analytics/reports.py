from __future__ import annotations

"""Generate human-readable reports from computed statistics."""

from typing import Any


class Reports:
    @staticmethod
    def mission_summary(stats: Any) -> str:
        if getattr(stats, "mission_completion", lambda: 0.0)() == 1.0:
            return "Mission Summary: Mission Complete"
        return "Mission Summary: Mission Incomplete"
