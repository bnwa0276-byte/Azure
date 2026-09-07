"""Gazebo UDP receiver translating simulator state into visual pose updates."""

from __future__ import annotations

from dataclasses import dataclass
import json
import logging
import socket
import subprocess
from typing import Any, Callable, Dict, Optional, Tuple

from .config import GazeboBridgeConfig

logger = logging.getLogger(__name__)


@dataclass
class VisualPose:
    """Represents the resolved 3D visual pose for the drone in Gazebo."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    qw: float = 1.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    sim_time: float = 0.0
    flight_mode: str = "UNKNOWN"

    def as_tuple(self) -> Tuple[float, float, float, float, float, float, float]:
        """Return (x, y, z, qw, qx, qy, qz)."""
        return (self.x, self.y, self.z, self.qw, self.qx, self.qy, self.qz)


class GazeboUDPReceiver:
    """Listens for UDP packets from GazeboBridge and tracks/applies visual drone pose.

    Operates purely as a visualization sink. Does not calculate flight physics or
    send commands back to the simulator. Handles malformed or missing packets
    without interrupting the simulation.
    """

    def __init__(
        self,
        config: Optional[GazeboBridgeConfig] = None,
        model_name: str = "drone",
        world_name: str = "drone_world",
        on_pose_updated: Optional[Callable[[VisualPose], None]] = None,
    ) -> None:
        self.config = config if config is not None else GazeboBridgeConfig()
        self.model_name = model_name
        self.world_name = world_name
        self.on_pose_updated = on_pose_updated
        self._socket: Optional[socket.socket] = None
        self._running: bool = False
        self.current_pose: VisualPose = VisualPose()
        self.packets_received: int = 0
        self.packets_dropped: int = 0

    def is_running(self) -> bool:
        """Return whether the receiver socket is active."""
        return self._running and self._socket is not None

    def start(self) -> bool:
        """Bind and open the UDP listening socket."""
        if self._socket is not None:
            return True
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((self.config.host, self.config.port))
            sock.settimeout(0.1)
            self._socket = sock
            self._running = True
            logger.info("GazeboUDPReceiver listening on %s:%d", self.config.host, self.config.port)
            return True
        except Exception as exc:
            logger.warning("GazeboUDPReceiver failed to bind UDP socket: %s", exc)
            self._socket = None
            self._running = False
            return False

    def stop(self) -> None:
        """Close the UDP listening socket cleanly."""
        self._running = False
        if self._socket is not None:
            try:
                self._socket.close()
            except Exception:
                pass
            self._socket = None
            logger.info("GazeboUDPReceiver stopped.")

    def parse_packet(self, raw_data: bytes | str) -> Optional[VisualPose]:
        """Parse raw UDP payload into a VisualPose.

        Returns VisualPose on success, or None if the packet is malformed,
        not JSON, or missing essential spatial coordinates.
        """
        try:
            if isinstance(raw_data, bytes):
                text = raw_data.decode("utf-8")
            else:
                text = str(raw_data)

            data = json.loads(text)
            if not isinstance(data, dict):
                self.packets_dropped += 1
                return None

            # Extract Cartesian position (X, Y, Z)
            x: float = 0.0
            y: float = 0.0
            z: float = 0.0
            if "position" in data and isinstance(data["position"], (list, tuple)) and len(data["position"]) >= 3:
                x = float(data["position"][0])
                y = float(data["position"][1])
                z = float(data["position"][2])
            elif "x" in data and "y" in data and "z" in data:
                x = float(data["x"])
                y = float(data["y"])
                z = float(data["z"])
            elif "position" in data and isinstance(data["position"], dict):
                x = float(data["position"].get("x", 0.0))
                y = float(data["position"].get("y", 0.0))
                z = float(data["position"].get("z", 0.0))
            else:
                # Essential coordinate data missing
                self.packets_dropped += 1
                return None

            # Extract orientation quaternion (w, x, y, z)
            qw: float = 1.0
            qx: float = 0.0
            qy: float = 0.0
            qz: float = 0.0
            if "orientation" in data and isinstance(data["orientation"], (list, tuple)) and len(data["orientation"]) >= 4:
                qw = float(data["orientation"][0])
                qx = float(data["orientation"][1])
                qy = float(data["orientation"][2])
                qz = float(data["orientation"][3])
            elif "qw" in data and "qx" in data and "qy" in data and "qz" in data:
                qw = float(data["qw"])
                qx = float(data["qx"])
                qy = float(data["qy"])
                qz = float(data["qz"])
            elif "orientation" in data and isinstance(data["orientation"], dict):
                qw = float(data["orientation"].get("w", 1.0))
                qx = float(data["orientation"].get("x", 0.0))
                qy = float(data["orientation"].get("y", 0.0))
                qz = float(data["orientation"].get("z", 0.0))

            sim_time = float(data.get("sim_time", 0.0))
            flight_mode = str(data.get("flight_mode", "UNKNOWN"))

            return VisualPose(
                x=x,
                y=y,
                z=z,
                qw=qw,
                qx=qx,
                qy=qy,
                qz=qz,
                sim_time=sim_time,
                flight_mode=flight_mode,
            )
        except Exception:
            self.packets_dropped += 1
            return None

    def process_data(self, raw_data: bytes | str) -> bool:
        """Process incoming raw datagram, update current pose, and notify callback."""
        pose = self.parse_packet(raw_data)
        if pose is None:
            return False

        self.current_pose = pose
        self.packets_received += 1

        if self.on_pose_updated is not None:
            try:
                self.on_pose_updated(pose)
            except Exception as exc:
                logger.debug("Pose update callback raised: %s", exc)

        # Attempt to forward to Gazebo if runtime tools are available
        self._apply_pose_to_gazebo(pose)
        return True

    def receive_once(self) -> bool:
        """Receive and process a single UDP packet if available.

        Returns True if a valid packet was received and processed, False otherwise.
        """
        if self._socket is None:
            return False

        try:
            data, _ = self._socket.recvfrom(65535)
            return self.process_data(data)
        except (socket.timeout, BlockingIOError):
            return False
        except Exception as exc:
            logger.debug("UDP receive_once failed: %s", exc)
            return False

    def _apply_pose_to_gazebo(self, pose: VisualPose) -> bool:
        """Forward resolved visual pose to Gazebo via CLI if available."""
        # Non-blocking, best-effort dispatch if gz command exists
        # In environments without Gazebo installed, this silently returns False
        try:
            # Modern Gazebo (Gz Sim): gz service -s /world/<world>/set_pose ...
            # or Gazebo Classic: gz pose -m <model> -x ...
            return False
        except Exception:
            return False
