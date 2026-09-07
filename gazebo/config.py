"""Configuration for the Gazebo visualization bridge."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GazeboBridgeConfig:
    """Immutable configuration parameters for the Gazebo UDP bridge.

    Attributes:
        host: IP address or hostname of the Gazebo host (default 127.0.0.1).
        port: UDP port where Gazebo listens for pose packets (default 9871).
        enabled: Whether the bridge is active and transmitting packets.
        rate_limit_hz: Maximum packet transmission rate in Hertz.
        derive_yaw_from_velocity: Whether to derive visual heading from velocity.
    """

    host: str = "127.0.0.1"
    port: int = 9871
    enabled: bool = True
    rate_limit_hz: float = 50.0
    derive_yaw_from_velocity: bool = True

    def __post_init__(self) -> None:
        if not isinstance(self.host, str) or not self.host.strip():
            raise ValueError(f"host must be a non-empty string, got {self.host!r}")
        if not isinstance(self.port, int) or not (1 <= self.port <= 65535):
            raise ValueError(f"port must be an integer between 1 and 65535, got {self.port!r}")
        if not isinstance(self.rate_limit_hz, (int, float)) or self.rate_limit_hz <= 0.0:
            raise ValueError(f"rate_limit_hz must be a positive number, got {self.rate_limit_hz!r}")

        object.__setattr__(self, "host", str(self.host).strip())
        object.__setattr__(self, "port", int(self.port))
        object.__setattr__(self, "enabled", bool(self.enabled))
        object.__setattr__(self, "rate_limit_hz", float(self.rate_limit_hz))
        object.__setattr__(self, "derive_yaw_from_velocity", bool(self.derive_yaw_from_velocity))
