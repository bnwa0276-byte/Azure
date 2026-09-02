import unittest

from drone import Drone, FlightMode
from autopilot import PController
from flight_controller import FlightController


class AutopilotTests(unittest.TestCase):
    def test_altitude_acquisition(self) -> None:
        drone = Drone()
        # prepare flight
        drone.change_mode(FlightMode.INITIALIZING)
        drone.change_mode(FlightMode.IDLE)
        drone.change_mode(FlightMode.ARMED)
        # initialize at ground level via physics for deterministic behavior
        drone.physics.set_altitude(0.0)
        drone.step_physics(0.0)

        controller = FlightController(drone)
        controller.enable_altitude_hold(target_altitude=20.0, Kp=1.0)

        # run a simple control loop and assert we're closer to the target
        initial_error = abs(drone.altitude - 20.0)
        for _ in range(200):
            controller.update_altitude_hold(0.1)
            drone.step_physics(0.1)

        final_error = abs(drone.altitude - 20.0)
        self.assertLess(final_error, initial_error)

    def test_recovery_from_disturbance(self) -> None:
        drone = Drone()
        drone.change_mode(FlightMode.INITIALIZING)
        drone.change_mode(FlightMode.IDLE)
        drone.change_mode(FlightMode.ARMED)
        drone.physics.set_altitude(0.0)
        drone.step_physics(0.0)

        controller = FlightController(drone)
        controller.enable_altitude_hold(target_altitude=10.0, Kp=2.0)

        # run to reduce initial error
        for _ in range(60):
            controller.update_altitude_hold(0.1)
            drone.step_physics(0.1)

        # simulate sudden drop and ensure controller reduces the new error
        drone.physics.set_altitude(2.0)
        drone.step_physics(0.0)
        post_drop_error = abs(drone.altitude - 10.0)

        for _ in range(100):
            controller.update_altitude_hold(0.1)
            drone.step_physics(0.1)

        recovered_error = abs(drone.altitude - 10.0)
        self.assertLess(recovered_error, post_drop_error)

    def test_target_altitude_change(self) -> None:
        drone = Drone()
        drone.change_mode(FlightMode.INITIALIZING)
        drone.change_mode(FlightMode.IDLE)
        drone.change_mode(FlightMode.ARMED)
        drone.physics.set_altitude(0.0)
        drone.step_physics(0.0)

        controller = FlightController(drone)
        controller.enable_altitude_hold(target_altitude=5.0, Kp=1.0)

        for _ in range(60):
            controller.update_altitude_hold(0.1)
            drone.step_physics(0.1)

        # measure error to new target before changing it
        pre_change_error = abs(drone.altitude - 15.0)
        controller.autopilot.set_target(15.0)

        for _ in range(100):
            controller.update_altitude_hold(0.1)
            drone.step_physics(0.1)

        post_change_error = abs(drone.altitude - 15.0)
        self.assertLess(post_change_error, pre_change_error)

    def test_output_clamping(self) -> None:
        controller = PController(Kp=100.0, target_altitude=10.0, min_thrust=5.0, max_thrust=15.0)
        # large error produces a required thrust beyond limits; ensure clamped
        thrust = controller.update(0.0)
        self.assertLessEqual(thrust, 15.0)
        self.assertGreaterEqual(thrust, 5.0)


if __name__ == "__main__":
    unittest.main()
