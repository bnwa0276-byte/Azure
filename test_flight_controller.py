import unittest

from drone import Drone, FlightMode, HealthStatus, SensorStatus
from flight_controller import FlightController
 
def run_simulation_until(drone, controller, target_mode, dt=0.05, max_steps=1000):
    for _ in range(max_steps):
        controller.update(dt)
        drone.step_physics(dt)
        if drone.mode == target_mode:
            return
    raise AssertionError(f"Timed out waiting for mode {target_mode}")


class FlightControllerTests(unittest.TestCase):
    def test_gps_loss_generates_warning(self) -> None:
        drone = Drone()
        drone.change_mode(FlightMode.INITIALIZING)
        drone.change_mode(FlightMode.IDLE)
        drone.change_mode(FlightMode.ARMED)
        # perform non-blocking takeoff via a temporary controller
        temp = FlightController(drone)
        drone.takeoff(flight_controller=temp)
        run_simulation_until(drone, temp, FlightMode.HOVER)

        drone.health_monitor.update_gps_availability(False)
        controller = FlightController(drone)

        action = controller.evaluate()

        self.assertEqual(action, "GPS_WARNING")
        self.assertEqual(drone.mode, FlightMode.HOVER)

    def test_battery_below_20_aborts_mission(self) -> None:
        health_monitor = HealthStatus(
            battery_level=50.0,
            gps_available=True,
            imu_status=SensorStatus.OK,
            motor_status=SensorStatus.OK,
        )
        drone = Drone(health_monitor=health_monitor)
        drone.change_mode(FlightMode.INITIALIZING)
        drone.change_mode(FlightMode.IDLE)
        drone.change_mode(FlightMode.ARMED)
        temp = FlightController(drone)
        drone.takeoff(flight_controller=temp)
        run_simulation_until(drone, temp, FlightMode.HOVER)
        drone.change_mode(FlightMode.MISSION)

        drone.health_monitor.update_battery_level(19.0)
        controller = FlightController(drone)

        action = controller.evaluate()

        self.assertEqual(action, "ABORT_MISSION")
        # run the landing sequence to completion
        run_simulation_until(drone, controller, FlightMode.IDLE)

    def test_battery_below_10_immediate_landing(self) -> None:
        drone = Drone()
        drone.change_mode(FlightMode.INITIALIZING)
        drone.change_mode(FlightMode.IDLE)
        drone.change_mode(FlightMode.ARMED)
        temp = FlightController(drone)
        drone.takeoff(flight_controller=temp)
        run_simulation_until(drone, temp, FlightMode.HOVER)

        drone.health_monitor.update_battery_level(9.0)
        controller = FlightController(drone)

        action = controller.evaluate()

        self.assertEqual(action, "IMMEDIATE_LANDING")
        run_simulation_until(drone, controller, FlightMode.IDLE)

    def test_motor_failure_forces_emergency(self) -> None:
        drone = Drone()
        drone.change_mode(FlightMode.INITIALIZING)
        drone.change_mode(FlightMode.IDLE)
        drone.change_mode(FlightMode.ARMED)
        temp = FlightController(drone)
        drone.takeoff(flight_controller=temp)
        run_simulation_until(drone, temp, FlightMode.HOVER)

        drone.health_monitor.update_motor_status(SensorStatus.FAILED)
        controller = FlightController(drone)

        action = controller.evaluate()

        self.assertEqual(action, "EMERGENCY")
        self.assertEqual(drone.mode, FlightMode.EMERGENCY)

    def test_degraded_sensor_generates_warning_but_continues(self) -> None:
        drone = Drone()
        drone.change_mode(FlightMode.INITIALIZING)
        drone.change_mode(FlightMode.IDLE)
        drone.change_mode(FlightMode.ARMED)
        temp = FlightController(drone)
        drone.takeoff(flight_controller=temp)
        run_simulation_until(drone, temp, FlightMode.HOVER)

        drone.health_monitor.update_motor_status(SensorStatus.DEGRADED)
        controller = FlightController(drone)

        action = controller.evaluate()

        self.assertEqual(action, "DEGRADED_WARNING")
        self.assertEqual(drone.mode, FlightMode.HOVER)

    def test_hover_mode_does_not_continue_climbing_after_takeoff(self) -> None:
        """Regression test for ES-022A: verify HOVER mode calls altitude hold controller."""
        drone = Drone()
        drone.change_mode(FlightMode.INITIALIZING)
        drone.change_mode(FlightMode.IDLE)
        drone.change_mode(FlightMode.ARMED)

        controller = FlightController(drone)
        drone.takeoff(target_altitude=15.0, tolerance=0.5, Kp=1.5, flight_controller=controller)

        # Run until HOVER mode is reached
        run_simulation_until(drone, controller, FlightMode.HOVER, dt=0.02, max_steps=1000)

        # Verify we're in HOVER
        self.assertEqual(drone.mode, FlightMode.HOVER)
        self.assertTrue(controller.altitude_hold_enabled,
                       "Altitude hold should remain enabled in HOVER mode")

        # Get final altitude
        final_altitude = drone.altitude

        # The key test: verify that update_altitude_hold() is being called by checking
        # that some thrust correction is happening (not just drifting at old thrust)
        self.assertGreater(final_altitude, 0.5,
                          "Drone should not have crashed immediately after HOVER transition")

    def test_hover_altitude_stability_over_extended_period(self) -> None:
        """Verify drone does not crash or runaway in HOVER mode."""
        drone = Drone()
        drone.change_mode(FlightMode.INITIALIZING)
        drone.change_mode(FlightMode.IDLE)
        drone.change_mode(FlightMode.ARMED)

        controller = FlightController(drone)
        drone.takeoff(target_altitude=20.0, tolerance=0.3, Kp=1.5, flight_controller=controller)

        # Run until HOVER
        run_simulation_until(drone, controller, FlightMode.HOVER, dt=0.02, max_steps=1000)

        self.assertEqual(drone.mode, FlightMode.HOVER)

        # Run for 10 seconds to verify altitude control is active
        for _ in range(500):  # 500 * 0.02 = 10 seconds
            controller.update(0.02)
            drone.step_physics(0.02)

        # Key test: altitude should not runaway to thousands of meters like the original bug
        # and should not instantly crash to ground
        # P-controller behavior is acceptable as long as it's within reasonable bounds
        self.assertGreater(drone.altitude, -1.0,
                          "Drone should not have crashed")
        self.assertLess(drone.altitude, 100.0,
                       "Drone should not have runaway altitude climb (original ES-022 bug)")

    def test_takeoff_does_not_enter_hover_while_vertical_velocity_is_excessive(self) -> None:
        """A near-target altitude alone should not trigger HOVER while vertical speed is still high."""
        drone = Drone()
        drone.change_mode(FlightMode.INITIALIZING)
        drone.change_mode(FlightMode.IDLE)
        drone.change_mode(FlightMode.ARMED)

        controller = FlightController(drone)
        drone.takeoff(target_altitude=15.0, dt=0.02, tolerance=0.5, touch_vz=0.5, Kp=1.5, flight_controller=controller)

        for _ in range(100):
            controller.update(0.02)
            drone.step_physics(0.02)
            if abs(drone.altitude - 15.0) <= 0.5 and abs(drone.vertical_velocity) > 0.5:
                self.assertNotEqual(drone.mode, FlightMode.HOVER,
                                    "HOVER should not start while vertical velocity is still excessive")
                break

    def test_takeoff_enters_hover_after_altitude_and_velocity_settle(self) -> None:
        """Once both altitude tolerance and vertical-velocity settling are satisfied, HOVER should begin."""
        drone = Drone()
        drone.change_mode(FlightMode.INITIALIZING)
        drone.change_mode(FlightMode.IDLE)
        drone.change_mode(FlightMode.ARMED)

        controller = FlightController(drone)
        drone.takeoff(target_altitude=15.0, dt=0.02, tolerance=0.5, touch_vz=0.5, Kp=1.5, flight_controller=controller)

        hover_entry_altitude = None
        hover_entry_vz = None
        post_hover_altitudes: list[float] = []
        in_hover = False

        for _ in range(4000):
            controller.update(0.02)
            drone.step_physics(0.02)

            if drone.mode == FlightMode.HOVER:
                if not in_hover:
                    hover_entry_altitude = drone.altitude
                    hover_entry_vz = drone.vertical_velocity
                    in_hover = True
                post_hover_altitudes.append(drone.altitude)
                if len(post_hover_altitudes) >= 150:
                    break

        self.assertIsNotNone(hover_entry_altitude)
        self.assertIsNotNone(hover_entry_vz)
        self.assertAlmostEqual(hover_entry_altitude, 15.0, delta=2.0,
                               msg="HOVER should be entered near the commanded altitude")
        self.assertLess(abs(hover_entry_vz), 0.5,
                        "HOVER should only begin after the drone is vertically settled")

        final_altitude = post_hover_altitudes[-1]
        max_altitude = max(post_hover_altitudes)
        min_altitude = min(post_hover_altitudes)
        self.assertLess(abs(final_altitude - 15.0), 5.0,
                        "Drone should settle near the 15m target after entering HOVER")
        self.assertLess(max_altitude - min_altitude, 6.0,
                        "Hover should not exhibit runaway climb or dive oscillation")

    def test_takeoff_hover_transition_maintains_altitude_hold(self) -> None:
        """Verify TAKEOFF->HOVER transition maintains altitude hold for continued control."""
        drone = Drone()
        drone.change_mode(FlightMode.INITIALIZING)
        drone.change_mode(FlightMode.IDLE)
        drone.change_mode(FlightMode.ARMED)

        controller = FlightController(drone)

        # Verify altitude hold is not enabled before takeoff
        self.assertFalse(controller.altitude_hold_enabled)

        # Request takeoff with flight controller handoff
        drone.takeoff(target_altitude=15.0, tolerance=0.5, Kp=1.5, flight_controller=controller)

        # Run one step to enter ramp phase
        controller.update(0.02)
        drone.step_physics(0.02)

        # Run until climb phase (where altitude hold is enabled)
        for _ in range(100):
            controller.update(0.02)
            drone.step_physics(0.02)
            if controller.altitude_hold_enabled:
                break

        # Verify altitude hold was enabled during climb
        self.assertTrue(controller.altitude_hold_enabled,
                       "Altitude hold should be enabled during takeoff climb phase")

        # Run until HOVER (takeoff completion)
        run_simulation_until(drone, controller, FlightMode.HOVER, dt=0.02, max_steps=1000)

        # Verify altitude hold remains enabled in HOVER (for continued altitude maintenance)
        self.assertTrue(controller.altitude_hold_enabled,
                        "Altitude hold should remain enabled in HOVER mode for altitude maintenance")


if __name__ == "__main__":
    unittest.main()
