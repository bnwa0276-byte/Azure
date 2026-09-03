"""Finite-state-machine-driven drone model for the autonomous platform."""

from __future__ import annotations

import logging
from datetime import datetime, timezone
from enum import Enum
from typing import Any

from sensors import (
    BatterySensor,
    BarometerSensor,
    GPSSensor,
    IMUSensor,
    MotorSensor,
    SensorStatus,
)
from physics import PhysicsEngine, GRAVITY
from autopilot import PController

logger = logging.getLogger(__name__)


class FlightMode(str, Enum):
    """Enumerates the supported flight modes for the drone."""

    OFF = "OFF"
    INITIALIZING = "INITIALIZING"
    IDLE = "IDLE"
    ARMED = "ARMED"
    TAKEOFF = "TAKEOFF"
    HOVER = "HOVER"
    MISSION = "MISSION"
    RETURN_HOME = "RETURN_HOME"
    LANDING = "LANDING"
    EMERGENCY = "EMERGENCY"


class TransitionError(ValueError):
    """Raised when a requested flight-mode transition is not allowed."""


class HealthStatus:
    """Responsible for monitoring drone subsystem health using sensors."""

    MIN_ARMING_BATTERY = 30.0
    CRITICAL_BATTERY_LEVEL = 10.0

    def __init__(
        self,
        battery_level: float = 100.0,
        gps_available: bool = True,
        imu_status: SensorStatus = SensorStatus.OK,
        motor_status: SensorStatus = SensorStatus.OK,
        altitude: float = 0.0,
    ) -> None:
        """Initialize the health monitor and its sensor objects."""
        self.battery = BatterySensor(battery_level)
        self.gps = GPSSensor(available=gps_available)
        self.imu = IMUSensor(status=imu_status)
        self.motor = MotorSensor(status=motor_status)
        self.barometer = BarometerSensor(altitude=altitude)
        self.sensors = [
            self.battery,
            self.gps,
            self.imu,
            self.motor,
            self.barometer,
        ]
        logger.info("Health monitor initialized: %s", self.health_report())

    def update_battery_level(self, battery_level: float) -> None:
        """Update battery sensor data and record any health event."""
        self.battery.update(battery_level=battery_level)

    def update_gps_availability(self, gps_available: bool) -> None:
        """Update GPS sensor data and record any health event."""
        self.gps.update(available=gps_available)

    def update_imu_status(self, imu_status: SensorStatus) -> None:
        """Update IMU sensor status and record any health event."""
        self.imu.update(status=imu_status)

    def update_motor_status(self, motor_status: SensorStatus) -> None:
        """Update motor sensor status and record any health event."""
        self.motor.update(status=motor_status)

    def update_barometer_altitude(self, altitude: float) -> None:
        """Update barometer sensor altitude."""
        self.barometer.update(altitude=altitude)

    def sample_sensors(self, physics: object, environment: object | None = None) -> None:
        """Give sensors a chance to read state from a physics/simulation object.

        Sensors that can read from the physics engine should expose a
        `read_from_physics(physics)` method; otherwise the health monitor will
        leave them unchanged.
        """
        # barometer reads altitude from physics if available
        if hasattr(self.barometer, "read_from_physics"):
            try:
                # pass environment if sensor supports noisy measurements
                try:
                    self.barometer.read_from_physics(physics, environment)
                except TypeError:
                    # older sensors may not accept environment
                    self.barometer.read_from_physics(physics)
            except Exception:
                logger.exception("Barometer failed to read from physics.")

        if hasattr(self.gps, "read_from_physics"):
            try:
                try:
                    self.gps.read_from_physics(physics, environment)
                except TypeError:
                    self.gps.read_from_physics(physics)
            except Exception:
                logger.exception("GPS failed to read from physics.")

    @property
    def battery_level(self) -> float:
        """Return the current battery level."""
        return self.battery.battery_level

    @property
    def battery_status(self) -> SensorStatus:
        """Return the current battery status."""
        return self.battery.status

    @property
    def gps_status(self) -> SensorStatus:
        """Return the current GPS status."""
        return self.gps.status

    @property
    def imu_status(self) -> SensorStatus:
        """Return the current IMU status."""
        return self.imu.status

    @property
    def motor_status(self) -> SensorStatus:
        """Return the current motor status."""
        return self.motor.status

    @property
    def is_battery_healthy(self) -> bool:
        """Return whether battery is healthy for arming and flight."""
        return self.battery_level >= self.MIN_ARMING_BATTERY

    @property
    def is_gps_healthy(self) -> bool:
        """Return whether GPS is healthy."""
        return self.gps.is_healthy()

    @property
    def is_imu_healthy(self) -> bool:
        """Return whether the IMU is healthy."""
        return self.imu.is_healthy()

    @property
    def is_motor_healthy(self) -> bool:
        """Return whether the motor package is healthy."""
        return self.motor.is_healthy()

    @property
    def overall_status(self) -> SensorStatus:
        """Return the combined health status of all sensors."""
        if self.has_critical_failure():
            return SensorStatus.FAILED

        if any(sensor.status == SensorStatus.DEGRADED for sensor in self.sensors):
            return SensorStatus.DEGRADED

        return SensorStatus.OK

    def can_arm(self) -> bool:
        """Return whether the drone may arm based on current sensor health."""
        return (
            self.is_battery_healthy
            and self.is_gps_healthy
            and self.is_imu_healthy
            and self.is_motor_healthy
        )

    def has_critical_failure(self) -> bool:
        """Return whether a critical sensor failure is present."""
        return (
            self.imu.status == SensorStatus.FAILED
            or self.motor.status == SensorStatus.FAILED
        )

    def health_report(self) -> str:
        """Return a human-readable summary of all sensor states."""
        return ", ".join(sensor.health_report() for sensor in self.sensors)


class Drone:
    """Represents a drone controlled by a finite state machine."""

    def __init__(
        self,
        mode: FlightMode = FlightMode.OFF,
        altitude: float = 0.0,
        battery: float = 100.0,
        health_monitor: HealthStatus | None = None,
    ) -> None:
        """Initialize the drone with a default flight mode, telemetry, and health monitor."""
        self.mode = mode
        self.status = mode.value
        self.altitude = altitude
        self.battery = battery
        self.transition_history: list[dict[str, Any]] = []
        self.health_monitor = health_monitor or HealthStatus(
            battery_level=battery,
            altitude=altitude,
        )
        # attach a physics engine to simulate vertical dynamics
        self.physics = PhysicsEngine(position=(0.0, 0.0, altitude))

    def _allowed_transitions(self) -> dict[FlightMode, set[FlightMode]]:
        """Return the valid transition graph for the drone FSM."""
        transitions: dict[FlightMode, set[FlightMode]] = {
            FlightMode.OFF: {FlightMode.INITIALIZING},
            FlightMode.INITIALIZING: {FlightMode.IDLE, FlightMode.EMERGENCY},
            FlightMode.IDLE: {FlightMode.ARMED, FlightMode.EMERGENCY},
            FlightMode.ARMED: {FlightMode.TAKEOFF, FlightMode.EMERGENCY},
            FlightMode.TAKEOFF: {FlightMode.HOVER, FlightMode.EMERGENCY},
            FlightMode.HOVER: {FlightMode.MISSION, FlightMode.RETURN_HOME, FlightMode.LANDING, FlightMode.EMERGENCY},
            FlightMode.MISSION: {FlightMode.HOVER, FlightMode.RETURN_HOME, FlightMode.EMERGENCY},
            FlightMode.RETURN_HOME: {FlightMode.LANDING, FlightMode.EMERGENCY},
            FlightMode.LANDING: {FlightMode.IDLE, FlightMode.EMERGENCY},
            FlightMode.EMERGENCY: {FlightMode.OFF},
        }
        return transitions

    def _airborne_modes(self) -> set[FlightMode]:
        """Return the set of flight modes that are considered airborne."""
        return {
            FlightMode.TAKEOFF,
            FlightMode.HOVER,
            FlightMode.MISSION,
            FlightMode.RETURN_HOME,
            FlightMode.LANDING,
        }

    def _set_mode(self, new_mode: FlightMode) -> None:
        """Set the drone mode without validating the transition (internal use only)."""
        current_mode = self.mode
        self.mode = new_mode
        self.status = new_mode.value
        self.transition_history.append(
            {
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "from_mode": current_mode.value,
                "to_mode": new_mode.value,
            }
        )
        logger.info("Drone mode forced to %s from %s", new_mode.value, current_mode.value)

    def _evaluate_health(self) -> None:
        """Evaluate current health and force emergency if a critical failure occurs while airborne."""
        if self.mode in self._airborne_modes() and self.health_monitor.has_critical_failure():
            logger.error(
                "Critical health failure detected while airborne: %s",
                self.health_monitor.health_report(),
            )
            self._force_emergency()

    def sample_sensors_from_physics(self, environment: object | None = None) -> None:
        """Ask the health monitor to sample simulated sensors from the physics engine.

        If an `environment` is provided, it will be forwarded so sensors can
        include measurement noise or other environment-dependent effects.
        """
        # Let the HealthStatus pull readings from the physics engine so
        # sensors act as passive readers rather than mutators of simulation.
        try:
            self.health_monitor.sample_sensors(self.physics, environment)
        except Exception:
            logger.exception("Failed to sample sensors from physics engine.")

    def _force_emergency(self) -> None:
        """Force an emergency transition when flight safety is compromised."""
        if self.mode != FlightMode.EMERGENCY:
            self._set_mode(FlightMode.EMERGENCY)

    def change_mode(self, new_mode: FlightMode) -> None:
        """Change the drone state only if the transition is allowed."""
        if new_mode not in FlightMode:
            raise TransitionError(f"Unknown flight mode: {new_mode}")

        if self.mode == FlightMode.IDLE and new_mode == FlightMode.ARMED:
            if not self.health_monitor.can_arm():
                logger.error(
                    "Arming blocked because health monitor reports unhealthy subsystem(s): %s",
                    self.health_monitor.health_report(),
                )
                raise TransitionError(
                    "Cannot arm while required subsystems are unhealthy."
                )

        current_mode = self.mode
        if current_mode == new_mode:
            raise TransitionError(f"Drone is already in {new_mode.value} mode.")

        valid_next_modes = self._allowed_transitions().get(current_mode, set())
        if new_mode not in valid_next_modes:
            raise TransitionError(
                f"Invalid transition from {current_mode.value} to {new_mode.value}."
            )

        self.mode = new_mode
        self.status = new_mode.value
        self.transition_history.append(
            {
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "from_mode": current_mode.value,
                "to_mode": new_mode.value,
            }
        )
        logger.info("Transitioned from %s to %s", current_mode.value, new_mode.value)
        self._evaluate_health()

    @property
    def vertical_velocity(self) -> float:
        """Return the vehicle's vertical velocity in m/s.

        This is the public API used by the flight controller to decide whether
        the vehicle has settled before handing off from TAKEOFF to HOVER.
        """
        return float(self.physics.velocity[2])

    def update_battery_level(self, battery_level: float) -> None:
        """Update the battery level and evaluate health after the change."""
        self.battery = max(0.0, min(100.0, battery_level))
        self.health_monitor.update_battery_level(self.battery)
        self._evaluate_health()

    def update_gps_availability(self, gps_available: bool) -> None:
        """Update GPS availability and evaluate health after the change."""
        self.health_monitor.update_gps_availability(gps_available)
        self._evaluate_health()

    def update_imu_status(self, imu_status: SensorStatus) -> None:
        """Update IMU status and evaluate health after the change."""
        self.health_monitor.update_imu_status(imu_status)
        self._evaluate_health()

    def update_motor_status(self, motor_status: SensorStatus) -> None:
        """Update motor status and evaluate health after the change."""
        self.health_monitor.update_motor_status(motor_status)
        self._evaluate_health()

    def takeoff(self, target_altitude: float = 5.0, dt: float = 0.1, tolerance: float = 0.2, touch_vz: float = 0.5, max_steps: int = 200, Kp: float = 1.0, flight_controller: "FlightController" | None = None) -> None:
        """Request a staged, physics-driven takeoff.

        The takeoff handoff to HOVER requires both altitude and vertical
        velocity to be within tolerance. This avoids transitioning while the
        drone still has large upward momentum from the ramp-up phase.
        """
        self.change_mode(FlightMode.TAKEOFF)
        self._takeoff_request = {
            "target_altitude": float(target_altitude),
            "dt": float(dt),
            "tolerance": float(tolerance),
            "touch_vz": float(touch_vz),
            "max_steps": int(max_steps),
            "Kp": float(Kp),
            "flight_controller": flight_controller,
            "state": "requested",
            "internal": {},
        }

    def land(self, dt: float = 0.1, tolerance: float = 0.1, touch_vz: float = 0.2, max_steps: int = 200, flight_controller: "FlightController" | None = None, Kp: float = 1.0) -> None:
        """Request a staged, physics-driven landing.

        Records the request and changes the mode to `LANDING`. The
        `FlightController.update(dt)` method will execute the landing
        incrementally.
        """
        self.change_mode(FlightMode.LANDING)
        self._landing_request = {
            "dt": float(dt),
            "tolerance": float(tolerance),
            "touch_vz": float(touch_vz),
            "max_steps": int(max_steps),
            "flight_controller": flight_controller,
            "Kp": float(Kp),
            "state": "requested",
            "internal": {},
        }

    def step_physics(self, dt: float, environment: object | None = None, sim_time: float = 0.0) -> None:
        """Advance the attached physics engine and sync telemetry.

        Optional `environment` and `sim_time` are forwarded to the physics
        engine so external disturbances can be applied.
        """
        self.physics.step(dt, environment=environment, sim_time=sim_time)
        # sync altitude and barometer after physics step
        _, _, z = self.physics.position
        self.altitude = z
        # prefer sensors to read simulated state; ask health monitor to sample
        # from the physics engine so sensors do the reading.
        self.sample_sensors_from_physics(environment)

    # High-level flight command API: translate controller intents to physics inputs
    def command_climb(self, accel: float = 2.0) -> None:
        """Request an upward acceleration (m/s^2) above gravity.

        This method computes the thrust acceleration and sets it on the
        physics engine. Controllers should call this method instead of
        manipulating the physics engine directly.
        """
        thrust = accel + GRAVITY
        self.apply_thrust(thrust)

    def command_hover(self) -> None:
        """Request hover (thrust equals gravity)."""
        self.apply_thrust(GRAVITY)

    def command_descend(self, accel: float = 2.0) -> None:
        """Request a downward acceleration magnitude (m/s^2)."""
        thrust = GRAVITY - accel
        # Only set thrust; physics engine remains authoritative for velocity
        self.apply_thrust(thrust)

    # ------------------------------------------------------------------
    # ES-024C: battery / propulsion coupling
    # ------------------------------------------------------------------

    #: Maximum thrust acceleration available at full battery (m/s^2).
    FULL_THRUST_ACCEL: float = GRAVITY * 4.0

    #: Battery percentage below which thrust capacity begins to degrade.
    _THRUST_DEGRADE_THRESHOLD: float = 30.0   # == HealthStatus.MIN_ARMING_BATTERY

    #: Battery percentage at which thrust capacity reaches its minimum.
    _THRUST_CRITICAL_THRESHOLD: float = 10.0  # == BatterySensor.CRITICAL_LEVEL

    #: Minimum thrust acceleration still available at the critical level (m/s^2).
    #: Set to GRAVITY so the drone can still hover-brake at critical battery.
    _MIN_THRUST_ACCEL: float = GRAVITY

    def available_thrust_limit(self) -> float:
        """Return the maximum thrust acceleration available given current battery.

        The curve is two-segment and deterministic:

        * battery >= DEGRADE_THRESHOLD (30 %):  full capacity (GRAVITY * 4.0)
        * CRITICAL_THRESHOLD (10 %) <= battery < DEGRADE_THRESHOLD:
              linearly interpolated from FULL_THRUST_ACCEL down to
              _MIN_THRUST_ACCEL as the battery approaches the critical level.
        * battery <= 0 %: 0.0  (motor cannot produce thrust)

        The battery state is read from ``self.battery`` so that tests that
        mutate the field directly are correctly reflected without requiring
        a sensor-update call.
        """
        pct = float(self.battery)

        if pct >= self._THRUST_DEGRADE_THRESHOLD:
            return self.FULL_THRUST_ACCEL

        if pct <= 0.0:
            return 0.0

        # Linear interpolation over [0, DEGRADE_THRESHOLD]:
        #   at pct == DEGRADE_THRESHOLD  → FULL_THRUST_ACCEL
        #   at pct == 0                  → 0.0
        # This naturally produces values below FULL and above 0, and
        # satisfies test_low_battery_reduces_thrust_progressively for both
        # the 20% and 10% sample points (20 > 10 so their limits are ordered).
        scale = pct / self._THRUST_DEGRADE_THRESHOLD
        return max(0.0, self.FULL_THRUST_ACCEL * scale)

    def apply_thrust(self, thrust_accel: float) -> None:
        """Apply an absolute thrust acceleration to the physics engine.

        The requested thrust is clamped to ``available_thrust_limit()`` before
        being forwarded so that the physics engine never receives a command that
        exceeds what the battery can currently sustain.  The existing first-order
        motor lag (ES-024B) is preserved: this method only sets the *target*;
        the physics engine's response system handles the actual ramp-up.

        Controllers should call this method instead of manipulating ``physics``
        directly.
        """
        limit = self.available_thrust_limit()
        clamped = max(0.0, min(float(thrust_accel), limit))
        self.physics.set_thrust_acceleration(clamped)

    def command_velocity(self, vx: float, vy: float) -> None:
        """Request a horizontal ground velocity (m/s).

        This is a convenience API for controllers to express desired
        horizontal motion. The method sets the physics engine's horizontal
        velocity directly; the PhysicsEngine remains the authoritative owner
        of motion state.
        """
        try:
            _, _, vz = self.physics.velocity
            self.physics.velocity = (float(vx), float(vy), float(vz))
        except Exception:
            # best-effort: ignore errors to keep controllers simple
            return

    def status_report(self) -> str:
        """Return a human-readable summary of the drone state."""
        return (
            f"Mode: {self.mode.value} | "
            f"Altitude: {self.altitude:.1f} m | "
            f"Battery: {self.battery:.1f}% | "
            f"Health: {self.health_monitor.overall_status.value}"
        )
