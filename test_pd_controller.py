import unittest

from drone import Drone, FlightMode
from flight_controller import FlightController
from autopilot import PController, PDController


class PDControllerTests(unittest.TestCase):
    def test_pd_reduces_overshoot(self) -> None:
        dt = 0.05
        target = 10.0

        # P controller
        drone_p = Drone()
        drone_p.change_mode(FlightMode.INITIALIZING)
        drone_p.change_mode(FlightMode.IDLE)
        drone_p.change_mode(FlightMode.ARMED)
        drone_p.physics.set_altitude(0.0)
        drone_p.step_physics(0.0)
        controller_p = FlightController(drone_p)
        controller_p.autopilot = PController(Kp=20.0, target_altitude=target, max_delta_thrust=100.0, max_thrust=200.0)
        controller_p.altitude_hold_enabled = True

        max_alt_p = -1e9
        for _ in range(200):
            controller_p.update_altitude_hold(dt)
            drone_p.step_physics(dt)
            max_alt_p = max(max_alt_p, drone_p.altitude)

        overshoot_p = max(0.0, max_alt_p - target)

        # PD controller
        drone_pd = Drone()
        drone_pd.change_mode(FlightMode.INITIALIZING)
        drone_pd.change_mode(FlightMode.IDLE)
        drone_pd.change_mode(FlightMode.ARMED)
        drone_pd.physics.set_altitude(0.0)
        drone_pd.step_physics(0.0)
        controller_pd = FlightController(drone_pd)
        controller_pd.autopilot = PDController(Kp=20.0, Kd=3.0, target_altitude=target, max_delta_thrust=100.0, max_thrust=200.0)
        controller_pd.altitude_hold_enabled = True

        max_alt_pd = -1e9
        for _ in range(200):
            controller_pd.update_altitude_hold(dt)
            drone_pd.step_physics(dt)
            max_alt_pd = max(max_alt_pd, drone_pd.altitude)

        overshoot_pd = max(0.0, max_alt_pd - target)

        self.assertLess(overshoot_pd, overshoot_p)

    def test_pd_settling_time_after_disturbance(self) -> None:
        dt = 0.05
        target = 8.0
        tol = 0.5

        # Setup PD controller and reach target
        drone = Drone()
        drone.change_mode(FlightMode.INITIALIZING)
        drone.change_mode(FlightMode.IDLE)
        drone.change_mode(FlightMode.ARMED)
        drone.physics.set_altitude(0.0)
        drone.step_physics(0.0)

        controller = FlightController(drone)
        controller.autopilot = PDController(Kp=20.0, Kd=4.0, target_altitude=target, max_delta_thrust=100.0, max_thrust=200.0)
        controller.altitude_hold_enabled = True

        for _ in range(200):
            controller.update_altitude_hold(dt)
            drone.step_physics(dt)

        # apply disturbance
        drone.physics.set_altitude(max(0.0, drone.altitude - 4.0))
        drone.step_physics(0.0)

        # measure settling time to within tolerance
        steps = 0
        while steps < 400:
            controller.update_altitude_hold(dt)
            drone.step_physics(dt)
            steps += 1
            if abs(drone.altitude - target) <= tol:
                break

        self.assertLess(steps, 300)


if __name__ == "__main__":
    unittest.main()
