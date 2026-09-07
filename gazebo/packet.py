"""Packet structures and serialization for Gazebo visualization."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Any, Optional, Tuple


def compute_yaw_quaternion(
    vx: float,
    vy: float,
    min_speed: float = 0.1,
) -> Tuple[float, float, float, float]:
    """Compute an orientation quaternion (w, x, y, z) from horizontal velocity.

    The current 3-DoF simulator does not model angular dynamics. Roll and pitch
    are zero (level flight). When horizontal speed exceeds min_speed, visual yaw
    is derived from the horizontal motion vector. Otherwise, identity
    orientation is returned.

    Returns:
        Quaternion tuple in (w, x, y, z) order.
    """
    speed_sq = vx * vx + vy * vy
    if speed_sq > (min_speed * min_speed):
        yaw = math.atan2(vy, vx)
        half_yaw = yaw * 0.5
        qw = math.cos(half_yaw)
        qx = 0.0
        qy = 0.0
        qz = math.sin(half_yaw)
        return (qw, qx, qy, qz)
    return (1.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class GazeboPosePacket:
    """Immutable representation of simulated vehicle pose for Gazebo visualization.

    Attributes:
        sim_time: Authoritative simulation time in seconds.
        position: (x, y, z) Cartesian coordinates in meters (ENU).
        velocity: (vx, vy, vz) linear velocities in m/s.
        orientation: (w, x, y, z) quaternion.
        flight_mode: String representation of current flight mode.
        acceleration: Optional (ax, ay, az) linear accelerations in m/s^2.
    """

    sim_time: float
    position: Tuple[float, float, float]
    velocity: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float]
    flight_mode: str
    acceleration: Optional[Tuple[float, float, float]] = None

    def to_dict(self) -> dict[str, Any]:
        """Convert packet to a dictionary representation."""
        data: dict[str, Any] = {
            "sim_time": float(self.sim_time),
            "position": [float(self.position[0]), float(self.position[1]), float(self.position[2])],
            "velocity": [float(self.velocity[0]), float(self.velocity[1]), float(self.velocity[2])],
            "acceleration": (
                [float(self.acceleration[0]), float(self.acceleration[1]), float(self.acceleration[2])]
                if self.acceleration is not None
                else None
            ),
            "orientation": [
                float(self.orientation[0]),
                float(self.orientation[1]),
                float(self.orientation[2]),
                float(self.orientation[3]),
            ],
            "flight_mode": str(self.flight_mode),
            "x": float(self.position[0]),
            "y": float(self.position[1]),
            "z": float(self.position[2]),
            "vx": float(self.velocity[0]),
            "vy": float(self.velocity[1]),
            "vz": float(self.velocity[2]),
            "qw": float(self.orientation[0]),
            "qx": float(self.orientation[1]),
            "qy": float(self.orientation[2]),
            "qz": float(self.orientation[3]),
        }
        if self.acceleration is not None:
            data["ax"] = float(self.acceleration[0])
            data["ay"] = float(self.acceleration[1])
            data["az"] = float(self.acceleration[2])
        else:
            data["ax"] = None
            data["ay"] = None
            data["az"] = None
        return data


def create_gazebo_packet(
    sim_time: float,
    position: Tuple[float, float, float],
    velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    acceleration: Optional[Tuple[float, float, float]] = None,
    flight_mode: str = "UNKNOWN",
    derive_yaw: bool = True,
) -> GazeboPosePacket:
    """Create a GazeboPosePacket from raw simulation state values.

    Direct ENU coordinate identity is preserved:
        Gazebo X = simulator X
        Gazebo Y = simulator Y
        Gazebo Z = simulator Z
    """
    px, py, pz = float(position[0]), float(position[1]), float(position[2])
    vx, vy, vz = float(velocity[0]), float(velocity[1]), float(velocity[2])

    acc: Optional[Tuple[float, float, float]] = None
    if acceleration is not None:
        acc = (float(acceleration[0]), float(acceleration[1]), float(acceleration[2]))

    if derive_yaw:
        orientation = compute_yaw_quaternion(vx, vy)
    else:
        orientation = (1.0, 0.0, 0.0, 0.0)

    return GazeboPosePacket(
        sim_time=float(sim_time),
        position=(px, py, pz),
        velocity=(vx, vy, vz),
        orientation=orientation,
        flight_mode=str(flight_mode),
        acceleration=acc,
    )


def serialize_packet(packet: GazeboPosePacket) -> bytes:
    """Serialize a GazeboPosePacket to UTF-8 JSON bytes."""
    return json.dumps(packet.to_dict()).encode("utf-8")


def deserialize_packet(data: bytes | str) -> GazeboPosePacket:
    """Deserialize UTF-8 JSON bytes or string into a GazeboPosePacket."""
    if isinstance(data, bytes):
        raw_str = data.decode("utf-8")
    else:
        raw_str = data
    d = json.loads(raw_str)
    pos = tuple(float(p) for p in d["position"])
    vel = tuple(float(v) for v in d["velocity"])
    orient = tuple(float(o) for o in d["orientation"])
    acc = tuple(float(a) for a in d["acceleration"]) if d.get("acceleration") is not None else None
    return GazeboPosePacket(
        sim_time=float(d["sim_time"]),
        position=(pos[0], pos[1], pos[2]),
        velocity=(vel[0], vel[1], vel[2]),
        orientation=(orient[0], orient[1], orient[2], orient[3]),
        flight_mode=str(d.get("flight_mode", "UNKNOWN")),
        acceleration=acc,
    )
