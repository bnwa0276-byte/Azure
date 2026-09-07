"""Simple physics engine for simulating drone vertical dynamics."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


GRAVITY = 9.81  # m/s^2


@dataclass
class PhysicsEngine:
    """Simulates simplified 3-DoF drone physics (x, y, z).

    Simulates vertical and horizontal dynamics with linear aerodynamic drag.

    Attributes:
        position: (x, y, z) in meters
        velocity: (vx, vy, vz) in m/s
        acceleration: (ax, ay, az) in m/s^2
        target_thrust_accel: commanded upward acceleration provided by thrust in m/s^2
        actual_thrust_accel: filtered thrust acceleration actually available at the current instant
        motor_time_constant: first-order motor response time constant in seconds
        thrust_accel: alias for the actual thrust acceleration used by the physics step
        drag_coeff_vertical: linear vertical aerodynamic drag coefficient in 1/s
        drag_coeff_horizontal: linear horizontal aerodynamic drag coefficient in 1/s
    """

    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    acceleration: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    target_thrust_accel: float = GRAVITY
    actual_thrust_accel: float = GRAVITY
    motor_time_constant: float = 0.1
    thrust_accel: float = GRAVITY
    drag_coeff_vertical: float = 0.0
    drag_coeff_horizontal: float = 0.0
    implicit_drag: bool = False

    def __post_init__(self) -> None:
        """Keep the public `thrust_accel` field aligned with the actual response state."""
        self.target_thrust_accel = float(self.target_thrust_accel)
        self.actual_thrust_accel = float(self.actual_thrust_accel)
        self.motor_time_constant = max(1e-6, float(self.motor_time_constant))
        self.thrust_accel = self.actual_thrust_accel
        if self.drag_coeff_vertical < 0.0:
            raise ValueError(f"drag_coeff_vertical must be non-negative, got {self.drag_coeff_vertical}")
        self.drag_coeff_vertical = float(self.drag_coeff_vertical)
        if self.drag_coeff_horizontal < 0.0:
            raise ValueError(f"drag_coeff_horizontal must be non-negative, got {self.drag_coeff_horizontal}")
        self.drag_coeff_horizontal = float(self.drag_coeff_horizontal)
        self.implicit_drag = bool(self.implicit_drag)

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

    def step(
        self,
        dt: float,
        environment: object | None = None,
        sim_time: float = 0.0,
        wind_velocity: Tuple[float, float] | Tuple[float, float, float] | None = None,
    ) -> None:
        """Advance the physics simulation by time step `dt` seconds.

        Aerodynamic drag is computed canonically by PhysicsEngine using relative
        air velocity (ES-024I):
            v_rel = v_vehicle - v_wind
            a_drag = -Cd * v_rel

        Args:
            dt: time step in seconds
            environment: optional environment providing external disturbances and wind
            sim_time: current simulation time in seconds
            wind_velocity: optional direct (wx, wy) or (wx, wy, wz) wind velocity vector
        """
        x, y, z = self.position
        vx, vy, vz = self.velocity

        # External non-aerodynamic acceleration (e.g. vertical turbulence)
        if environment is not None and hasattr(environment, "get_external_acceleration"):
            ex_ax, ex_ay, ex_az = environment.get_external_acceleration(self.position, self.velocity, sim_time)
        else:
            ex_ax = ex_ay = ex_az = 0.0

        # Environmental wind velocity vector (wx, wy)
        if wind_velocity is not None:
            wx = float(wind_velocity[0])
            wy = float(wind_velocity[1])
        elif environment is not None and hasattr(environment, "get_wind_velocity"):
            w = environment.get_wind_velocity(self.position, self.velocity, sim_time)
            wx = float(w[0])
            wy = float(w[1])
        else:
            wx = wy = 0.0

        # First-order motor response: actual thrust lags the commanded target
        # but is bounded to prevent unstable jumps for large dt values.
        response_factor = self._response_factor(dt)
        self.actual_thrust_accel += (self.target_thrust_accel - self.actual_thrust_accel) * response_factor
        self.thrust_accel = self.actual_thrust_accel

        # Determine canonical horizontal drag coefficient:
        # PhysicsEngine owns physical drag. If drag_coeff_horizontal is explicitly set (> 0.0),
        # it is authoritative. If it is 0.0 and an environment with drag_coef is supplied,
        # fallback to environment.drag_coef for backward compatibility with existing tests/callers.
        # Under NO circumstance are both applied.
        cd_h = self.drag_coeff_horizontal
        if cd_h == 0.0 and environment is not None and hasattr(environment, "drag_coef"):
            cd_h = max(0.0, float(getattr(environment, "drag_coef", 0.0)))

        # Compute net horizontal and vertical accelerations and update velocities
        if self.implicit_drag:
            # Backward-Euler (implicit) linear aerodynamic drag integration (ES-024K).
            # Unconditionally stable for any dt > 0 and Cd >= 0.
            # v_new = (v_old + dt * a_non_drag + Cd * dt * wind) / (1 + Cd * dt)
            if dt > 0.0:
                denom_h = 1.0 + cd_h * dt
                vx_new = (vx + ex_ax * dt + cd_h * dt * wx) / denom_h
                vy_new = (vy + ex_ay * dt + cd_h * dt * wy) / denom_h

                a_non_drag_z = self.actual_thrust_accel - GRAVITY + ex_az
                denom_v = 1.0 + self.drag_coeff_vertical * dt
                vz_new = (vz + a_non_drag_z * dt) / denom_v

                ax = (vx_new - vx) / dt
                ay = (vy_new - vy) / dt
                az = (vz_new - vz) / dt

                vx = vx_new
                vy = vy_new
                vz = vz_new
            else:
                ax = ex_ax - cd_h * (vx - wx)
                ay = ex_ay - cd_h * (vy - wy)
                az = self.actual_thrust_accel - GRAVITY - self.drag_coeff_vertical * vz + ex_az
        else:
            # Explicit Euler linear aerodynamic drag integration (legacy/default).
            # Compute net horizontal accelerations using relative air velocity:
            # v_rel = v_vehicle - v_wind
            # a_drag = -cd_h * v_rel
            # When vehicle velocity equals wind velocity, aerodynamic drag is zero.
            ax = ex_ax - cd_h * (vx - wx)
            ay = ex_ay - cd_h * (vy - wy)
            az = self.actual_thrust_accel - GRAVITY - self.drag_coeff_vertical * vz + ex_az

            # integrate velocity (semi-implicit Euler)
            vx = vx + ax * dt
            vy = vy + ay * dt
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
        self.acceleration = (ax, ay, az)
        self.velocity = (vx, vy, vz)
        self.position = (x, y, z)
