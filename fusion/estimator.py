"""Complementary filter based state estimator combining GPS, IMU, and barometer."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Tuple
import math


@dataclass
class EstimatorState:
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    confidence: float = 1.0
    diagnostics: dict = field(default_factory=dict)


class ComplementaryEstimator:
    """A simple complementary estimator that fuses IMU-integrated motion
    with low-rate absolute sensors (GPS and barometer).

    This is intentionally lightweight and modular so more advanced filters
    (EKF/UKF) can be introduced later with the same interface.
    """

    def __init__(self, alpha: float = 0.98, seed: Optional[int] = None) -> None:
        self.alpha = float(alpha)
        self.state = EstimatorState()
        self._last_update = 0.0
        self._gps_last_seen = -1.0
        self._has_initialized = False

    def reset(self) -> None:
        self.state = EstimatorState()
        self._last_update = 0.0
        self._gps_last_seen = -1.0
        self._has_initialized = False

    def initialize_from_sensors(self, gps_pos: Optional[Tuple[float, float, float]], baro_alt: Optional[float]) -> None:
        # prefer GPS for initial horizontal position, baro for altitude
        x = y = z = 0.0
        if gps_pos is not None:
            x, y, z = gps_pos
        elif baro_alt is not None:
            z = baro_alt
        self.state.position = (x, y, z)
        self.state.velocity = (0.0, 0.0, 0.0)
        self._has_initialized = True

    def update(self, dt: float, gps_pos: Optional[Tuple[float, float, float]], baro_alt: Optional[float], imu_accel: Optional[Tuple[float, float, float]], gps_available: bool = True) -> EstimatorState:
        """Advance estimator by dt seconds and incorporate available measurements.

        gps_pos: (x,y,z) absolute from GPS or None
        baro_alt: barometer altitude in meters or None
        imu_accel: measured acceleration (ax, ay, az) in m/s^2 or None
        gps_available: whether GPS is currently available
        """
        if not self._has_initialized:
            self.initialize_from_sensors(gps_pos, baro_alt)

        px, py, pz = self.state.position
        vx, vy, vz = self.state.velocity

        # integrate IMU acceleration for prediction
        if imu_accel is not None:
            ax, ay, az = imu_accel
            vx = vx + ax * dt
            vy = vy + ay * dt
            vz = vz + az * dt
            px = px + vx * dt
            py = py + vy * dt
            pz = pz + vz * dt

        # complementary corrections using GPS and barometer
        if gps_available and gps_pos is not None:
            gx, gy, gz = gps_pos
            px = self.alpha * px + (1.0 - self.alpha) * gx
            py = self.alpha * py + (1.0 - self.alpha) * gy
            # weak velocity correction from GPS
            vx = self.alpha * vx + (1.0 - self.alpha) * ((gx - px) / max(dt, 1e-6))
            vy = self.alpha * vy + (1.0 - self.alpha) * ((gy - py) / max(dt, 1e-6))
            self._gps_last_seen = 0.0
        else:
            # age gps-last-seen counter
            self._gps_last_seen += dt

        if baro_alt is not None:
            # complementary filter for altitude specifically
            pz = self.alpha * pz + (1.0 - self.alpha) * float(baro_alt)

        # update confidence: simple heuristic
        conf = 1.0
        if not gps_available:
            conf -= 0.4
        if imu_accel is None:
            conf -= 0.2
        conf = max(0.0, min(1.0, conf))

        self.state.position = (px, py, pz)
        self.state.velocity = (vx, vy, vz)
        self.state.confidence = conf
        self.state.diagnostics = {
            "gps_recent": self._gps_last_seen < 1.0,
            "alpha": self.alpha,
        }

        return self.state

    def get_state(self) -> EstimatorState:
        return self.state
