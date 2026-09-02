import unittest

from drone import Drone, FlightMode
from flight_controller import FlightController
from simulation_runner import SimulationRunner


class SimulationRunnerTests(unittest.TestCase):
    def test_timing_and_steps(self) -> None:
        drone = Drone()
        controller = FlightController(drone)
        runner = SimulationRunner(drone, controller, dt=0.02)
        runner.run(0.1)
        # 0.1s / 0.02s = 5 steps
        self.assertEqual(runner.step_count, 5)

    def test_event_order_takeoff_landing(self) -> None:
        drone = Drone()
        drone.change_mode(FlightMode.INITIALIZING)
        drone.change_mode(FlightMode.IDLE)
        drone.change_mode(FlightMode.ARMED)

        controller = FlightController(drone)
        runner = SimulationRunner(drone, controller, dt=0.05)

        # request takeoff (hand off to controller)
        drone.takeoff(target_altitude=3.0, dt=0.05, tolerance=0.5, flight_controller=controller, Kp=1.0)
        runner.run(5.0)

        # ensure takeoff requested and completed events exist and mode ended in HOVER
        names = [e["event"] for e in runner.event_log]
        self.assertIn("TAKEOFF_REQUESTED", names)
        self.assertIn("TAKEOFF_COMPLETED", names)
        self.assertEqual(drone.mode, FlightMode.HOVER)

        # now request landing and run
        drone.land(dt=0.05, tolerance=0.2, touch_vz=0.2, flight_controller=controller, Kp=1.0)
        runner.run(5.0)

        names = [e["event"] for e in runner.event_log]
        self.assertIn("LANDING_REQUESTED", names)
        self.assertIn("LANDING_COMPLETED", names)
        self.assertEqual(drone.mode, FlightMode.IDLE)


if __name__ == "__main__":
    unittest.main()
