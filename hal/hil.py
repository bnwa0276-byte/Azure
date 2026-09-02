from __future__ import annotations

"""Hardware-in-the-Loop bridge that connects HAL to transports and scheduler."""

from typing import Any, Optional, Callable
from hal.transports import TransportInterface, MockTransport
from hal.interfaces import VehicleInterface
from flight_recorder.recorder import FlightRecorder


class HILBridge:
    def __init__(self, vehicle: VehicleInterface, transport: TransportInterface, recorder: Optional[FlightRecorder] = None):
        self.vehicle = vehicle
        self.transport = transport
        self.recorder = recorder
        self._telemetry_cb: Optional[Callable[[Any], None]] = None
        # register receive callback to process incoming commands
        self.transport.register_receive_callback(self._on_receive)

    def _record(self, time: float, event: str, info: dict | None = None) -> None:
        if self.recorder is not None:
            self.recorder.record_event(time, event, info)

    def _on_receive(self, packet: Any, delivery_time: float) -> None:
        # Packet is expected to be a dict with 'cmd' key e.g. {'cmd':'velocity','vx':..., 'vy':...}
        self._record(delivery_time, "hil_receive", {"packet": packet})
        try:
            if isinstance(packet, dict):
                cmd = packet.get("cmd")
                if cmd == "velocity":
                    vx = float(packet.get("vx", 0.0))
                    vy = float(packet.get("vy", 0.0))
                    self.vehicle.command_velocity(vx, vy)
                elif cmd == "thrust":
                    a = float(packet.get("accel", 0.0))
                    self.vehicle.apply_thrust(a)
        except Exception as e:
            self._record(delivery_time, "hil_error", {"error": str(e)})

    def send_telemetry(self, now: float) -> None:
        # collect basic telemetry from vehicle and send over transport
        try:
            pos = self.vehicle.get_position()
        except Exception:
            pos = (0.0, 0.0, 0.0)
        try:
            alt = self.vehicle.get_altitude()
        except Exception:
            alt = 0.0
        packet = {"telemetry": {"position": pos, "altitude": alt, "time": now}}
        self.transport.send(packet, now)
        self._record(now, "hil_send", {"packet": packet})
