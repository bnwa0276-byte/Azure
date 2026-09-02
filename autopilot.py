"""Autopilot controllers for altitude hold.

This module provides a simple PController implementing a proportional
altitude hold. The controller computes a thrust command (absolute upward
acceleration in m/s^2) suitable for passing directly to the vehicle's
physics engine. The proportional law is `delta = Kp * error` and the
returned thrust is `GRAVITY + delta` so that zero error produces hover.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from physics import GRAVITY


@dataclass
class PController:
    """Proportional controller for altitude hold.

    Attributes:
        Kp: proportional gain (output units are m/s^2 per meter error)
        target_altitude: desired altitude in meters
        min_thrust: minimum allowed thrust acceleration (m/s^2)
        max_thrust: maximum allowed thrust acceleration (m/s^2)
    """

    Kp: float = 1.0
    target_altitude: float = 0.0
    min_thrust: float = 0.0
    max_thrust: float = GRAVITY * 4.0
    last_output: Optional[float] = None
    # limit how much the thrust command may change between updates (m/s^2)
    max_delta_thrust: float = 1.0

    def set_target(self, altitude: float) -> None:
        self.target_altitude = float(altitude)

    def update(self, current_altitude: float, dt: Optional[float] = None) -> float:
        """Compute and return a thrust command (absolute acceleration).

        The proportional term computes a delta acceleration which is added
        to gravity so that zero error corresponds to hover.
        """
        error = float(self.target_altitude) - float(current_altitude)
        delta = self.Kp * error
        thrust = GRAVITY + delta
        # clamp to safety limits
        thrust = max(self.min_thrust, min(self.max_thrust, thrust))

        # rate-limit sudden thrust changes to improve stability
        if self.last_output is None:
            self.last_output = GRAVITY

        desired = thrust
        delta_thrust = desired - self.last_output
        if delta_thrust > self.max_delta_thrust:
            thrust = self.last_output + self.max_delta_thrust
        elif delta_thrust < -self.max_delta_thrust:
            thrust = self.last_output - self.max_delta_thrust
        else:
            thrust = desired

        # final clamp (safety)
        thrust = max(self.min_thrust, min(self.max_thrust, thrust))
        self.last_output = thrust
        return thrust

    def get_last_output(self) -> Optional[float]:
        return self.last_output


@dataclass
class PDController(PController):
    """Proportional-Derivative controller for altitude hold.

    This controller extends `PController` by adding a derivative term that
    damps oscillations and reduces overshoot. The control law is:

        thrust = GRAVITY + Kp*error + Kd*(error_dot)

    where `error_dot` is the time derivative of the altitude error (m/s).

    Attributes:
        Kd: derivative gain (m/s^2 per (m/s) of error rate)
    """

    Kd: float = 0.1
    last_error: Optional[float] = None

    def update(self, current_altitude: float, dt: Optional[float] = None) -> float:
        error = float(self.target_altitude) - float(current_altitude)

        # default timestep fallback when not provided; matches common sim rate
        if dt is None:
            dt = 1.0 / 50.0

        # compute derivative of error
        error_dot = 0.0
        if self.last_error is not None:
            error_dot = (error - self.last_error) / float(dt)

        delta = self.Kp * error + self.Kd * error_dot
        thrust = GRAVITY + delta

        # clamp to safety limits
        thrust = max(self.min_thrust, min(self.max_thrust, thrust))

        # rate-limit thrust changes
        if self.last_output is None:
            self.last_output = GRAVITY

        desired = thrust
        delta_thrust = desired - self.last_output
        if delta_thrust > self.max_delta_thrust:
            thrust = self.last_output + self.max_delta_thrust
        elif delta_thrust < -self.max_delta_thrust:
            thrust = self.last_output - self.max_delta_thrust
        else:
            thrust = desired

        thrust = max(self.min_thrust, min(self.max_thrust, thrust))
        self.last_output = thrust
        self.last_error = error
        return thrust
