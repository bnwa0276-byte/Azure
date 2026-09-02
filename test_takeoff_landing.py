import unittest

from drone import Drone, FlightMode
from flight_controller import FlightController


class TakeoffLandingTests(unittest.TestCase):
    def test_takeoff_progression_and_altitude_hold_handoff(self) -> None:
        drone = Drone()
        drone.change_mode(FlightMode.INITIALIZING)
        drone.change_mode(FlightMode.IDLE)
        drone.change_mode(FlightMode.ARMED)

        fc = FlightController(drone)

        # ensure operational code does not call set_altitude during takeoff
        called = {"set_altitude": False}

        original_set_alt = drone.physics.set_altitude

        def forbidden_set_alt(a):
            called["set_altitude"] = True
            raise AssertionError("Operational code must not call set_altitude during takeoff")

        drone.physics.set_altitude = forbidden_set_alt

        try:
            drone.takeoff(target_altitude=6.0, dt=0.05, tolerance=0.5, flight_controller=fc, Kp=1.0)
            # run the controller loop until hover
            for _ in range(400):
                fc.update(0.05)
                drone.step_physics(0.05)
                if drone.mode == FlightMode.HOVER:
                    break
        finally:
            drone.physics.set_altitude = original_set_alt

        self.assertFalse(called["set_altitude"])
        self.assertEqual(drone.mode, FlightMode.HOVER)
        self.assertTrue(fc.altitude_hold_enabled)
        self.assertAlmostEqual(drone.altitude, 6.0, delta=1.0)

    def test_landing_progression_and_touchdown(self) -> None:
        drone = Drone()
        drone.change_mode(FlightMode.INITIALIZING)
        drone.change_mode(FlightMode.IDLE)
        drone.change_mode(FlightMode.ARMED)

        # place the drone at an altitude via the physics API (tests may set sim state)
        drone.physics.set_altitude(8.0)
        drone.step_physics(0.0)
        self.assertGreater(drone.altitude, 7.0)

        # move FSM into an airborne mode so landing is allowed
        drone.change_mode(FlightMode.TAKEOFF)
        drone.change_mode(FlightMode.HOVER)

        fc = FlightController(drone)

        # ensure operational code does not call set_altitude during landing
        called = {"set_altitude": False}
        original_set_alt = drone.physics.set_altitude

        def forbidden_set_alt(a):
            called["set_altitude"] = True
            raise AssertionError("Operational code must not call set_altitude during landing")

        drone.physics.set_altitude = forbidden_set_alt

        try:
            drone.land(dt=0.05, tolerance=0.2, touch_vz=0.2, flight_controller=fc, Kp=1.0)
            # run controller until idle
            for _ in range(400):
                fc.update(0.05)
                drone.step_physics(0.05)
                if drone.mode == FlightMode.IDLE:
                    break
        finally:
            drone.physics.set_altitude = original_set_alt

        self.assertFalse(called["set_altitude"])
        self.assertEqual(drone.mode, FlightMode.IDLE)
        self.assertLessEqual(drone.altitude, 0.5)


if __name__ == "__main__":
    unittest.main()
