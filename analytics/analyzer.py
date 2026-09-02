from __future__ import annotations

"""Analyzer: load flight logs and present them for processing."""

from typing import Iterable, List, Optional
from flight_recorder.recorder import FlightRecordEntry, FlightRecorder, Replay


class Analyzer:
    def __init__(self, recorder: Optional[FlightRecorder] = None, entries: Optional[Iterable[FlightRecordEntry]] = None):
        if recorder is not None:
            self.entries = list(recorder.entries())
        elif entries is not None:
            self.entries = list(entries)
        else:
            self.entries = []

    def replay(self) -> Replay:
        return Replay(self.entries)

    def get_entries(self) -> List[FlightRecordEntry]:
        return list(self.entries)
