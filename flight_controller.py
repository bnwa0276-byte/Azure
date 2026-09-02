"""Flight controller decision engine for the autonomous drone."""

from __future__ import annotations

import logging
from typing import Optional

from drone import Drone, FlightMode
from navigation import NavigationSystem
from sensors import SensorStatus
from autopilot import PController, PDController
from physics import GRAVITY

logger = logging.getLogger(__name__)


class FlightController:
    """Supervises the drone and decides actions based on flight state and health."""

    WARNING_BATTERY_LEVEL = 20.0
    IMMEDIATE_LANDING_LEVEL = 10.0

    def __init__(
        self,
        drone: "Drone",
        navigation_system: Optional[NavigationSystem] = None,
    ) -> None:
        self.drone = drone
        self.navigation_system = navigation_system
        self.autopilot: PController | None = None
        self.altitude_hold_enabled = False
        # guidance consumer does not compute geometry; expects GuidanceCommand
        # to be provided by an external GuidanceSystem.

    def supervise_mission(
        self,
        latitude: float,
        longitude: float,
        altitude: float,
    ) -> str:
        """Supervise mission progression using the navigation subsystem."""
        if self.navigation_system is None:
            return "NO_NAVIGATION"

        result = self.navigation_system.update_position(latitude, longitude, altitude)
        logger.info("Navigation result: %s", result)
        if result == "MISSION_COMPLETE":
            logger.info("Mission complete according to navigation system.")
        return result

    def evaluate(self) -> str:
        """Evaluate drone health and mode, then decide the next action."""
        health = self.drone.health_monitor
        mode = self.drone.mode

        if health.motor_status == SensorStatus.FAILED or health.imu_status == SensorStatus.FAILED:
            self._force_emergency()
            return "EMERGENCY"

        if health.battery_level <= self.IMMEDIATE_LANDING_LEVEL:
            self._force_landing()
            return "IMMEDIATE_LANDING"

        if health.battery_level <= self.WARNING_BATTERY_LEVEL:
            if mode == FlightMode.MISSION:
                self._initiate_landing()
                return "ABORT_MISSION"
            return "BATTERY_WARNING"

        if health.gps_status == SensorStatus.FAILED:
            logger.warning("GPS loss detected; staying in current mode with warning.")
            return "GPS_WARNING"

        if health.overall_status == SensorStatus.DEGRADED:
            logger.warning("Degraded sensor state detected; continuing flight with caution.")
            return "DEGRADED_WARNING"

        return "NO_ACTION"

    def _initiate_landing(self) -> None:
        if self.drone.mode == FlightMode.MISSION:
            self.drone.change_mode(FlightMode.RETURN_HOME)

        if self.drone.mode == FlightMode.RETURN_HOME:
            self.drone.land()
            logger.info("Mission aborted: initiating landing.")

    def _force_landing(self) -> None:
        if self.drone.mode == FlightMode.MISSION:
            self.drone.change_mode(FlightMode.RETURN_HOME)

        if self.drone.mode == FlightMode.TAKEOFF:
            self.drone.change_mode(FlightMode.HOVER)

        if self.drone.mode not in {FlightMode.LANDING, FlightMode.EMERGENCY}:
            self.drone.land()
            logger.info("Immediate landing initiated due to critical battery.")

    def _force_emergency(self) -> None:
        if self.drone.mode != FlightMode.EMERGENCY:
            self.drone.change_mode(FlightMode.EMERGENCY)
            logger.info("Emergency triggered due to critical subsystem failure.")

    # High-level flight commands: these do not perform physics themselves;
    # they instruct the physics engine via the Drone's `physics` object.
    def climb(self, accel: float = 2.0) -> None:
        """Command the vehicle to climb by increasing thrust above gravity.

        `accel` is the additional upward acceleration (m/s^2) above gravity.
        """
        # Delegate physics calculations to the Drone so controller remains
        # free of physics math and only expresses intent.
        self.drone.command_climb(accel)

    def hover(self) -> None:
        """Command the vehicle to hover (thrust equals gravity)."""
        self.drone.command_hover()

    def descend(self, accel: float = 2.0) -> None:
        """Command the vehicle to descend by reducing thrust below gravity.

        `accel` is the downward acceleration magnitude (m/s^2).
        """
        self.drone.command_descend(accel)

    def enable_altitude_hold(self, target_altitude: float, Kp: float = 1.0, min_thrust: float = 0.0, max_thrust: float = 39.24) -> None:
        """Enable altitude-hold mode using a PController.

        `max_thrust` default is roughly 4*g (g~9.81).
        """
        self.autopilot = PController(Kp=Kp, target_altitude=target_altitude, min_thrust=min_thrust, max_thrust=max_thrust)
        self.altitude_hold_enabled = True

    def disable_altitude_hold(self) -> None:
        self.autopilot = None
        self.altitude_hold_enabled = False

    def update_altitude_hold(self, dt: Optional[float] = None) -> None:
        """Compute a thrust command from the autopilot and apply it via Drone."""
        if not self.altitude_hold_enabled or self.autopilot is None:
            return
        # pass timestep to autopilot when available to allow derivative control
        thrust = self.autopilot.update(self.drone.altitude, dt)
        # delegate applying thrust to Drone so controller does not manipulate physics
        self.drone.apply_thrust(thrust)

    def apply_guidance(self, command: object) -> None:
        """Apply a guidance command produced by `GuidanceSystem`.

        The controller simply delegates horizontal velocity assignment to the
        `Drone` via `command_velocity`. It does not perform waypoint geometry
        calculations — those are handled by the guidance subsystem.
        """
        try:
            vx = float(getattr(command, "vx", 0.0))
            vy = float(getattr(command, "vy", 0.0))
        except Exception:
            return
        self.drone.command_velocity(vx, vy)

    def update(self, dt: float) -> None:
        """Run a single controller update step (non-blocking).

        This method executes staged takeoff/landing procedures when the
        `Drone` has requested them via `takeoff()` / `land()`. It delegates
        thrust application to the `Drone` and uses the `PController` for
        climb/descent guidance when needed.
        """
        # Evaluate high-level health first (may trigger emergency/landing)
        self.evaluate()

        # Handle takeoff request if present
        req = getattr(self.drone, "_takeoff_request", None)
        if req is not None and req.get("state") != "completed":
            internal = req.setdefault("internal", {})
            stage = internal.get("stage", "init")

            if stage == "init":
                internal["thrust"] = GRAVITY * 0.9
                internal["ramp_step"] = 0.5
                internal["lift_vz_threshold"] = 0.05
                internal["stage"] = "idle"

            if internal["stage"] == "idle":
                # apply idle thrust for one step then move to ramp
                self.drone.apply_thrust(internal["thrust"])
                internal["stage"] = "ramp"
                return

            if internal["stage"] == "ramp":
                internal["thrust"] = min(GRAVITY * 4.0, internal["thrust"] + internal["ramp_step"])
                self.drone.apply_thrust(internal["thrust"])
                vz = self.drone.physics.velocity[2]
                if vz > internal["lift_vz_threshold"] or self.drone.altitude > 0.05:
                    internal["stage"] = "climb"
                    # prepare climb controller
                    if req.get("flight_controller") is self:
                        self.autopilot = PDController(
                            Kp=req["Kp"],
                            Kd=2.0,
                            target_altitude=req["target_altitude"],
                            min_thrust=0.0,
                            max_thrust=GRAVITY * 4.0,
                        )
                        self.altitude_hold_enabled = True
                        internal["handoff"] = True
                    else:
                        internal["p"] = PController(Kp=req["Kp"], target_altitude=req["target_altitude"], min_thrust=0.0, max_thrust=GRAVITY * 4.0)
                return

            if internal["stage"] == "climb":
                if internal.get("handoff"):
                    # flight controller's autopilot handles thrust
                    self.update_altitude_hold(dt)
                else:
                    thrust = internal["p"].update(self.drone.altitude)
                    self.drone.apply_thrust(thrust)

                altitude_error = abs(self.drone.altitude - req["target_altitude"])
                vertical_velocity = abs(self.drone.vertical_velocity)
                if altitude_error <= req["tolerance"] and vertical_velocity <= req.get("touch_vz", 0.5):
                    # Only transition to HOVER once altitude is near target and the
                    # vehicle has settled vertically. This prevents the unstable
                    # handoff caused by large upward momentum from the climb ramp.
                    self.drone.battery = max(0.0, self.drone.battery - 10.0)
                    self.drone.health_monitor.update_battery_level(self.drone.battery)
                    self.drone.change_mode(FlightMode.HOVER)
                    req["state"] = "completed"
                    return

        # Handle landing requests
        lreq = getattr(self.drone, "_landing_request", None)
        if lreq is not None and lreq.get("state") != "completed":
            internal = lreq.setdefault("internal", {})
            stage = internal.get("stage", "init")

            if stage == "init":
                # prepare descent controller or handoff
                internal["stage"] = "descent"
                if lreq.get("flight_controller") is self:
                    self.enable_altitude_hold(target_altitude=0.0, Kp=lreq.get("Kp", 1.0))
                    internal["handoff"] = True
                else:
                    internal["p"] = PController(Kp=lreq.get("Kp", 1.0), target_altitude=0.0, min_thrust=0.0, max_thrust=GRAVITY * 4.0)

            if internal["stage"] == "descent":
                if internal.get("handoff"):
                    self.update_altitude_hold(dt)
                else:
                    thrust = internal["p"].update(self.drone.altitude)
                    self.drone.apply_thrust(thrust)

                vz = abs(self.drone.physics.velocity[2])
                if self.drone.altitude <= lreq["tolerance"] and vz <= lreq["touch_vz"]:
                    # touchdown: stop motors and transition to IDLE
                    self.drone.apply_thrust(0.0)
                    self.drone.battery = max(0.0, self.drone.battery - 5.0)
                    self.drone.health_monitor.update_battery_level(self.drone.battery)
                    self.drone.change_mode(FlightMode.IDLE)
                    lreq["state"] = "completed"
                    # disable altitude hold if we enabled it
                    if internal.get("handoff"):
                        self.disable_altitude_hold()
                    return

        # Maintain altitude hold during HOVER mode (allows smooth transition from TAKEOFF)
        if self.drone.mode == FlightMode.HOVER and self.altitude_hold_enabled:
            self.update_altitude_hold(dt)
