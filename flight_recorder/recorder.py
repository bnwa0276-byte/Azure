"""Flight data recorder: captures telemetry, events, controller outputs, and environment snapshots."""
from __future__ import annotations

from dataclasses import dataclass, field, asdict
from typing import Any, Dict, List, Optional, Iterable
import csv
import io


@dataclass
class FlightRecordEntry:
    sim_time: float
    telemetry: Dict[str, Any]
    events: List[Dict[str, Any]] = field(default_factory=list)
    controller: Optional[Dict[str, Any]] = None
    environment: Optional[Dict[str, Any]] = None


class FlightRecorder:
    """Collects stepwise flight data in memory and exports it.

    The recorder is read-only with respect to the simulation: callers provide
    data to be recorded; the recorder does not mutate the simulation state.
    """

    def __init__(self) -> None:
        self._entries: List[FlightRecordEntry] = []
        self._events: List[Dict[str, Any]] = []

    def record_event(self, time: float, name: str, info: Optional[Dict[str, Any]] = None) -> None:
        self._events.append({"time": float(time), "event": name, "info": info or {}})

    def record_step(self, sim_time: float, telemetry: Dict[str, Any], controller: Optional[Dict[str, Any]] = None, environment: Optional[Dict[str, Any]] = None) -> None:
        # snapshot events since last step
        events = list(self._events)
        self._events.clear()
        entry = FlightRecordEntry(sim_time=float(sim_time), telemetry=dict(telemetry), events=events, controller=dict(controller) if controller is not None else None, environment=dict(environment) if environment is not None else None)
        self._entries.append(entry)

    def entries(self) -> Iterable[FlightRecordEntry]:
        return list(self._entries)

    def clear(self) -> None:
        self._entries.clear()
        self._events.clear()

    def export_csv(self, fp) -> None:
        """Export recorded entries to a CSV file-like object or path.

        `fp` may be a file path (str) or a file-like object with a `write()` method.
        """
        close_when_done = False
        if isinstance(fp, str):
            f = open(fp, "w", newline="")
            close_when_done = True
        else:
            f = fp

        writer = csv.writer(f)
        # header
        writer.writerow(["sim_time", "mode", "altitude", "vz", "battery", "target_altitude", "events", "controller", "environment"])
        for e in self._entries:
            telemetry = e.telemetry or {}
            events = ";".join([ev.get("event", "") for ev in (e.events or [])])
            writer.writerow([
                f"{e.sim_time:.6f}",
                telemetry.get("mode", ""),
                telemetry.get("altitude", ""),
                telemetry.get("vz", ""),
                telemetry.get("battery", ""),
                telemetry.get("target_altitude", ""),
                events,
                e.controller or "",
                e.environment or "",
            ])

        if close_when_done:
            f.close()


class Replay:
    """Replay recorded entries via an iterator API."""

    def __init__(self, entries: Iterable[FlightRecordEntry]):
        self._entries = list(entries)
        self._index = 0

    def __iter__(self):
        return self

    def __next__(self) -> FlightRecordEntry:
        if self._index >= len(self._entries):
            raise StopIteration
        v = self._entries[self._index]
        self._index += 1
        return v

    def reset(self) -> None:
        self._index = 0
