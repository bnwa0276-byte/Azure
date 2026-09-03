"""Simple physics engine for simulating drone vertical dynamics."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


GRAVITY = 9.81  # m/s^2


@dataclass
class PhysicsEngine:
    """Simulates simplified 3-DoF drone physics (x, y, z).

    Only vertical dynamics (z) are meaningful for this simplified engine.

    Attributes:
        position: (x, y, z) in meters
        velocity: (vx, vy, vz) in m/s
        acceleration: (ax, ay, az) in m/s^2
        target_thrust_accel: commanded upward acceleration provided by thrust in m/s^2
        actual_thrust_accel: filtered thrust acceleration actually available at the current instant
        motor_time_constant: first-order motor response time constant in seconds
        thrust_accel: alias for the actual thrust acceleration used by the physics step
        drag_coeff_vertical: linear vertical aerodynamic drag coefficient in 1/s
    """

    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    acceleration: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    target_thrust_accel: float = GRAVITY
    actual_thrust_accel: float = GRAVITY
    motor_time_constant: float = 0.1
    thrust_accel: float = GRAVITY
    drag_coeff_vertical: float = 0.0

    def __post_init__(self) -> None:
        """Keep the public `thrust_accel` field aligned with the actual response state."""
        self.target_thrust_accel = float(self.target_thrust_accel)
        self.actual_thrust_accel = float(self.actual_thrust_accel)
        self.motor_time_constant = max(1e-6, float(self.motor_time_constant))
        self.thrust_accel = self.actual_thrust_accel
        if self.drag_coeff_vertical < 0.0:
            raise ValueError(f"drag_coeff_vertical must be non-negative, got {self.drag_coeff_vertical}")
        self.drag_coeff_vertical = float(self.drag_coeff_vertical)

    def _response_factor(self, dt: float) -> float:
        """Compute a stable first-order response factor for a given timestep."""
        if dt <= 0.0:
            return 0.0
        return min(1.0, float(dt) / self.motor_time_constant)

    def set_thrust_acceleration(self, thrust_accel: float) -> None:
        """Set the commanded upward thrust acceleration (m/s^2).

        The physics engine uses a first-order motor response before the value
        is applied to the broader integrated dynamics. The controller remains
        the owner of the command decision, while the physics engine remains the
        owner of the actual physical state update.
        """
        self.target_thrust_accel = float(thrust_accel)

    def set_altitude(self, altitude: float) -> None:
        """Teleport the simulated vehicle to a specific altitude.

        This is an explicit operation for tests and initialization only; it
        updates position and zeroes vertical velocity to avoid implicit state
        mutation from external modules.
        """
        x, y, _ = self.position
        self.position = (x, y, max(0.0, float(altitude)))
        # reset vertical velocity to zero when teleporting
        vx, vy, _ = self.velocity
        self.velocity = (vx, vy, 0.0)

    def step(self, dt: float, environment: object | None = None, sim_time: float = 0.0) -> None:
        """Advance the physics simulation by time step `dt` seconds.

        If an environment is provided, the engine will query it for external
        accelerations and include them in the integration.
        """
        x, y, z = self.position
        vx, vy, vz = self.velocity

        # external acceleration
        if environment is not None and hasattr(environment, "get_external_acceleration"):
            ex_ax, ex_ay, ex_az = environment.get_external_acceleration(self.position, self.velocity, sim_time)
        else:
            ex_ax = ex_ay = ex_az = 0.0

        # First-order motor response: actual thrust lags the commanded target
        # but is bounded to prevent unstable jumps for large dt values.
        response_factor = self._response_factor(dt)
        self.actual_thrust_accel += (self.target_thrust_accel - self.actual_thrust_accel) * response_factor
        self.thrust_accel = self.actual_thrust_accel

        # compute net vertical acceleration (thrust minus gravity minus vertical drag plus external)
        az = self.actual_thrust_accel - GRAVITY - self.drag_coeff_vertical * vz + ex_az

        # integrate velocity and position (semi-implicit Euler)
        vx = vx + ex_ax * dt
        vy = vy + ex_ay * dt
        vz = vz + az * dt

        x = x + vx * dt
        y = y + vy * dt
        z = z + vz * dt

        # Unilateral zero-restitution ground contact boundary condition:
        # when at or below ground level and the ground supports the vehicle (az <= 0),
        # normal reaction force balances gravity so net vertical acceleration is zero
        if z <= 0.0:
            z = 0.0
            if az <= 0.0:
                az = 0.0
                vz = 0.0
            elif vz < 0.0:
                vz = 0.0

        # store back
        self.acceleration = (ex_ax, ex_ay, az)
        self.velocity = (vx, vy, vz)
        self.position = (x, y, z)
