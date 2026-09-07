"""Gazebo visualization bridge transmitting simulation state over UDP."""

from __future__ import annotations

import logging
import socket
from typing import Any, Dict, Optional, Sequence, Tuple

from .config import GazeboBridgeConfig
from .packet import GazeboPosePacket, create_gazebo_packet, serialize_packet

logger = logging.getLogger(__name__)


class GazeboBridge:
    """Read-only visualization sink conforming to VisualizerProtocol.

    Receives drone position, velocity, and telemetry from SimulationRunner and
    transmits kinematic pose updates to Gazebo via non-blocking UDP datagrams.
    """

    def __init__(self, config: Optional[GazeboBridgeConfig] = None) -> None:
        self.config = config if config is not None else GazeboBridgeConfig()
        self._socket: Optional[socket.socket] = None
        self._last_send_sim_time: float = -float("inf")
        self._prev_position: Optional[Tuple[float, float, float]] = None
        self._prev_sim_time: Optional[float] = None
        self._packets_sent: int = 0

    def is_connected(self) -> bool:
        """Return True if the bridge is enabled and has an open socket."""
        return self.config.enabled and self._socket is not None

    def connect(self) -> bool:
        """Initialize the non-blocking UDP socket.

        Returns True if successful, False if the bridge is disabled or socket
        creation fails.
        """
        if not self.config.enabled:
            return False

        if self._socket is not None:
            return True

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setblocking(False)
            self._socket = sock
            logger.info("GazeboBridge opened UDP socket for %s:%d", self.config.host, self.config.port)
            return True
        except Exception as exc:
            logger.warning("GazeboBridge failed to open UDP socket: %s", exc)
            self._socket = None
            return False

    def close(self) -> None:
        """Close the UDP socket cleanly."""
        if self._socket is not None:
            try:
                self._socket.close()
            except Exception:
                pass
            self._socket = None
            logger.info("GazeboBridge UDP socket closed.")

    def _is_rate_limited(self, sim_time: float) -> bool:
        """Check if sending should be throttled based on rate_limit_hz."""
        min_interval = 1.0 / self.config.rate_limit_hz
        if (sim_time - self._last_send_sim_time) < (min_interval - 1e-9):
            return True
        return False

    def update(
        self,
        *,
        telemetry: Dict[str, Any],
        position: Tuple[float, float, float],
        waypoints: Sequence[Any] = (),
        path: Sequence[Tuple[float, float, float]] = (),
    ) -> None:
        """VisualizerProtocol entrypoint called on each simulation step.

        Extracts state, constructs a GazeboPosePacket, and sends a UDP datagram
        if rate limiting allows. Never raises exceptions to the caller.
        """
        if not self.config.enabled:
            return

        sim_time = float(telemetry.get("sim_time", 0.0))

        # Check rate limiting
        if self._is_rate_limited(sim_time):
            self._prev_position = position
            self._prev_sim_time = sim_time
            return

        # Ensure socket is open
        if self._socket is None:
            if not self.connect():
                self._prev_position = position
                self._prev_sim_time = sim_time
                return

        # Resolve velocity: prefer telemetry, fallback to position delta
        velocity: Tuple[float, float, float]
        if "velocity" in telemetry and isinstance(telemetry["velocity"], (tuple, list)) and len(telemetry["velocity"]) >= 3:
            v = telemetry["velocity"]
            velocity = (float(v[0]), float(v[1]), float(v[2]))
        elif "vx" in telemetry and "vy" in telemetry:
            velocity = (
                float(telemetry["vx"]),
                float(telemetry["vy"]),
                float(telemetry.get("vz", 0.0)),
            )
        elif self._prev_position is not None and self._prev_sim_time is not None and (sim_time - self._prev_sim_time) > 1e-6:
            dt = sim_time - self._prev_sim_time
            vx = (position[0] - self._prev_position[0]) / dt
            vy = (position[1] - self._prev_position[1]) / dt
            vz = float(telemetry.get("vz", (position[2] - self._prev_position[2]) / dt))
            velocity = (vx, vy, vz)
        else:
            velocity = (0.0, 0.0, float(telemetry.get("vz", 0.0)))

        # Resolve acceleration if available through telemetry
        acceleration: Optional[Tuple[float, float, float]] = None
        if "acceleration" in telemetry and isinstance(telemetry["acceleration"], (tuple, list)) and len(telemetry["acceleration"]) >= 3:
            a = telemetry["acceleration"]
            acceleration = (float(a[0]), float(a[1]), float(a[2]))
        elif "ax" in telemetry and "ay" in telemetry and "az" in telemetry:
            acceleration = (
                float(telemetry["ax"]),
                float(telemetry["ay"]),
                float(telemetry["az"]),
            )

        flight_mode = str(telemetry.get("mode", "UNKNOWN"))

        # Build packet
        packet = create_gazebo_packet(
            sim_time=sim_time,
            position=position,
            velocity=velocity,
            acceleration=acceleration,
            flight_mode=flight_mode,
            derive_yaw=self.config.derive_yaw_from_velocity,
        )

        # Transmit UDP datagram safely
        try:
            payload = serialize_packet(packet)
            if self._socket is not None:
                self._socket.sendto(payload, (self.config.host, self.config.port))
                self._packets_sent += 1
                self._last_send_sim_time = sim_time
        except Exception as exc:
            # Network failures must never raise or interrupt the simulation
            logger.debug("GazeboBridge UDP sendto failed: %s", exc)

        self._prev_position = position
        self._prev_sim_time = sim_time
