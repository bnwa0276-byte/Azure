import unittest

from physics import PhysicsEngine, GRAVITY
from drone import Drone, FlightMode
from flight_controller import FlightController


class PhysicsEngineTests(unittest.TestCase):
    def test_hover_keeps_altitude(self) -> None:
        engine = PhysicsEngine()
        engine.set_thrust_acceleration(GRAVITY)
        engine.step(0.1)
        # hovering with zero initial velocity should keep z at 0
        self.assertAlmostEqual(engine.position[2], 0.0, places=6)

    def test_climb_increases_altitude(self) -> None:
        engine = PhysicsEngine()
        engine.set_thrust_acceleration(GRAVITY + 2.0)
        engine.step(1.0)
        self.assertGreater(engine.position[2], 0.0)

    def test_descend_prevents_negative_altitude(self) -> None:
        engine = PhysicsEngine(position=(0.0, 0.0, 1.0))
        engine.set_thrust_acceleration(0.0)  # large descent
        # step long enough to hit ground
        engine.step(1.0)
        self.assertGreaterEqual(engine.position[2], 0.0)

    def test_motor_thrust_lags_target(self) -> None:
        engine = PhysicsEngine(motor_time_constant=0.2)
        engine.set_thrust_acceleration(GRAVITY + 8.0)

        self.assertEqual(engine.target_thrust_accel, GRAVITY + 8.0)
        self.assertLess(engine.actual_thrust_accel, engine.target_thrust_accel)

    def test_motor_thrust_eventually_reaches_target(self) -> None:
        engine = PhysicsEngine(motor_time_constant=0.1)
        engine.set_thrust_acceleration(GRAVITY + 10.0)

        for _ in range(100):
            engine.step(0.02)

        self.assertAlmostEqual(engine.actual_thrust_accel, engine.target_thrust_accel, delta=0.5)

    def test_large_dt_does_not_create_unstable_thrust_response(self) -> None:
        engine = PhysicsEngine(motor_time_constant=0.25)
        engine.set_thrust_acceleration(GRAVITY + 12.0)
        engine.step(2.0)

        self.assertTrue(float("-inf") < engine.actual_thrust_accel < float("inf"))
        self.assertGreaterEqual(engine.actual_thrust_accel, 0.0)
        self.assertLessEqual(engine.actual_thrust_accel, engine.target_thrust_accel * 1.2)


class BatteryPropulsionTests(unittest.TestCase):
    def test_healthy_battery_keeps_full_thrust_capacity(self) -> None:
        drone = Drone(battery=100.0)
        self.assertAlmostEqual(drone.available_thrust_limit(), GRAVITY * 4.0)

    def test_low_battery_reduces_thrust_progressively(self) -> None:
        drone = Drone(battery=100.0)
        healthy = drone.available_thrust_limit()
        drone.battery = 35.0
        full_low = drone.available_thrust_limit()
        drone.battery = 20.0
        reduced = drone.available_thrust_limit()
        drone.battery = 10.0
        critical = drone.available_thrust_limit()

        self.assertAlmostEqual(healthy, GRAVITY * 4.0)
        self.assertAlmostEqual(full_low, GRAVITY * 4.0)
        self.assertLess(reduced, full_low)
        self.assertLess(critical, reduced)
        self.assertGreaterEqual(reduced, 0.0)

    def test_commanded_thrust_cannot_exceed_battery_limit(self) -> None:
        drone = Drone(battery=15.0)
        drone.apply_thrust(GRAVITY * 10.0)
        limit = drone.available_thrust_limit()

        self.assertLessEqual(drone.physics.target_thrust_accel, limit)
        self.assertGreaterEqual(drone.physics.target_thrust_accel, 0.0)

    def test_battery_limited_hover_remains_stable(self) -> None:
        drone = Drone(battery=100.0)
        drone.change_mode(FlightMode.INITIALIZING)
        drone.change_mode(FlightMode.IDLE)
        drone.change_mode(FlightMode.ARMED)
        drone.step_physics(0.0)
        drone.command_hover()

        for _ in range(50):
            drone.step_physics(0.1)

        self.assertAlmostEqual(drone.altitude, 0.0, delta=0.1)


class FlightControllerCommandTests(unittest.TestCase):
    def test_controller_climb_hover_descend(self) -> None:
        drone = Drone()
        drone.change_mode(FlightMode.INITIALIZING)
        drone.change_mode(FlightMode.IDLE)
        drone.change_mode(FlightMode.ARMED)
        temp = FlightController(drone)
        drone.takeoff(flight_controller=temp)
        # run until hover
        for _ in range(200):
            temp.update(0.05)
            drone.step_physics(0.05)
            if drone.mode == FlightMode.HOVER:
                break

        controller = FlightController(drone)

        # climb
        controller.climb(accel=3.0)
        drone.step_physics(0.5)
        self.assertGreater(drone.altitude, 0.0)

        # hover
        prev_alt = drone.altitude
        controller.hover()
        drone.step_physics(0.5)
        # altitude should remain near previous value
        self.assertAlmostEqual(drone.altitude, prev_alt, delta=5.0)

        # descend: allow multiple small physics steps for descent to begin
        controller.descend(accel=5.0)
        descended = False
        # allow up to 5 seconds of simulation for descent to begin
        for _ in range(50):
            drone.step_physics(0.1)
            if drone.altitude <= prev_alt:
                descended = True
                break
        self.assertTrue(descended, "Drone did not begin descent within expected time")


if __name__ == "__main__":
    unittest.main()
