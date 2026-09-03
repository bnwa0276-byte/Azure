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


class VerticalDragTests(unittest.TestCase):
    def test_zero_coefficient_preserves_existing_behavior(self) -> None:
        engine_default = PhysicsEngine(position=(0.0, 0.0, 5.0), velocity=(0.0, 0.0, 2.0))
        engine_explicit_zero = PhysicsEngine(position=(0.0, 0.0, 5.0), velocity=(0.0, 0.0, 2.0), drag_coeff_vertical=0.0)
        engine_default.step(0.1)
        engine_explicit_zero.step(0.1)
        self.assertEqual(engine_default.position, engine_explicit_zero.position)
        self.assertEqual(engine_default.velocity, engine_explicit_zero.velocity)
        self.assertEqual(engine_default.acceleration, engine_explicit_zero.acceleration)

    def test_upward_velocity_opposed_by_drag(self) -> None:
        engine_no_drag = PhysicsEngine(position=(0.0, 0.0, 10.0), velocity=(0.0, 0.0, 4.0), drag_coeff_vertical=0.0)
        engine_with_drag = PhysicsEngine(position=(0.0, 0.0, 10.0), velocity=(0.0, 0.0, 4.0), drag_coeff_vertical=0.5)

        engine_no_drag.step(0.1)
        engine_with_drag.step(0.1)

        # az without drag: 0.0; az with drag: -0.5 * 4.0 = -2.0
        self.assertAlmostEqual(engine_no_drag.acceleration[2], 0.0)
        self.assertAlmostEqual(engine_with_drag.acceleration[2], -2.0)
        self.assertLess(engine_with_drag.velocity[2], engine_no_drag.velocity[2])
        self.assertLess(engine_with_drag.position[2], engine_no_drag.position[2])

    def test_downward_velocity_opposed_by_drag(self) -> None:
        engine_no_drag = PhysicsEngine(position=(0.0, 0.0, 10.0), velocity=(0.0, 0.0, -4.0), drag_coeff_vertical=0.0)
        engine_with_drag = PhysicsEngine(position=(0.0, 0.0, 10.0), velocity=(0.0, 0.0, -4.0), drag_coeff_vertical=0.5)

        engine_no_drag.step(0.1)
        engine_with_drag.step(0.1)

        # az without drag: 0.0; az with drag: -0.5 * (-4.0) = +2.0
        self.assertAlmostEqual(engine_no_drag.acceleration[2], 0.0)
        self.assertAlmostEqual(engine_with_drag.acceleration[2], 2.0)
        self.assertGreater(engine_with_drag.velocity[2], engine_no_drag.velocity[2])
        self.assertGreater(engine_with_drag.position[2], engine_no_drag.position[2])

    def test_drag_is_zero_when_vz_is_zero(self) -> None:
        engine = PhysicsEngine(position=(0.0, 0.0, 10.0), velocity=(0.0, 0.0, 0.0), drag_coeff_vertical=1.0)
        engine.step(0.1)
        self.assertAlmostEqual(engine.acceleration[2], 0.0)
        self.assertAlmostEqual(engine.velocity[2], 0.0)
        self.assertAlmostEqual(engine.position[2], 10.0)

    def test_negative_drag_coeff_rejected(self) -> None:
        with self.assertRaises(ValueError):
            PhysicsEngine(drag_coeff_vertical=-0.5)

    def test_drag_sign_explicit_verification(self) -> None:
        coeff = 0.75
        vz_up = 2.0
        vz_down = -2.0

        eng_up = PhysicsEngine(position=(0.0, 0.0, 10.0), velocity=(0.0, 0.0, vz_up), drag_coeff_vertical=coeff)
        eng_up.step(0.01)
        # vz > 0 -> drag acceleration is negative (downward)
        self.assertLess(eng_up.acceleration[2], 0.0)
        self.assertAlmostEqual(eng_up.acceleration[2], -coeff * vz_up)

        eng_down = PhysicsEngine(position=(0.0, 0.0, 10.0), velocity=(0.0, 0.0, vz_down), drag_coeff_vertical=coeff)
        eng_down.step(0.01)
        # vz < 0 -> drag acceleration is positive (upward damping)
        self.assertGreater(eng_down.acceleration[2], 0.0)
        self.assertAlmostEqual(eng_down.acceleration[2], -coeff * vz_down)


class GroundContactTests(unittest.TestCase):
    def test_ground_resting_state_reports_zero_acceleration(self) -> None:
        engine = PhysicsEngine(
            position=(0.0, 0.0, 0.0),
            velocity=(0.0, 0.0, 0.0),
            target_thrust_accel=0.0,
            actual_thrust_accel=0.0,
        )
        engine.set_thrust_acceleration(0.0)
        engine.step(0.1)
        self.assertAlmostEqual(engine.position[2], 0.0)
        self.assertAlmostEqual(engine.velocity[2], 0.0)
        self.assertAlmostEqual(engine.acceleration[2], 0.0)

    def test_ground_resting_with_partial_thrust(self) -> None:
        partial_thrust = GRAVITY * 0.5
        engine = PhysicsEngine(
            position=(0.0, 0.0, 0.0),
            velocity=(0.0, 0.0, 0.0),
            target_thrust_accel=partial_thrust,
            actual_thrust_accel=partial_thrust,
        )
        engine.set_thrust_acceleration(partial_thrust)
        engine.step(0.1)
        self.assertAlmostEqual(engine.position[2], 0.0)
        self.assertAlmostEqual(engine.velocity[2], 0.0)
        self.assertAlmostEqual(engine.acceleration[2], 0.0)

    def test_ground_liftoff_when_thrust_exceeds_gravity(self) -> None:
        climb_thrust = GRAVITY + 4.0
        engine = PhysicsEngine(
            position=(0.0, 0.0, 0.0),
            velocity=(0.0, 0.0, 0.0),
            target_thrust_accel=climb_thrust,
            actual_thrust_accel=climb_thrust,
        )
        engine.set_thrust_acceleration(climb_thrust)
        engine.step(0.1)
        self.assertGreater(engine.acceleration[2], 0.0)
        self.assertGreater(engine.velocity[2], 0.0)
        self.assertGreater(engine.position[2], 0.0)

    def test_touchdown_resolves_to_static_contact(self) -> None:
        engine = PhysicsEngine(
            position=(0.0, 0.0, 0.5),
            velocity=(0.0, 0.0, -2.0),
            target_thrust_accel=0.0,
            actual_thrust_accel=0.0,
        )
        # Step until ground contact
        for _ in range(20):
            engine.step(0.05)
            if engine.position[2] <= 0.0:
                break

        # Assert touchdown resolves to z=0 and vz=0
        self.assertAlmostEqual(engine.position[2], 0.0)
        self.assertAlmostEqual(engine.velocity[2], 0.0)

        # Perform another step and assert it remains z=0, vz=0, az=0 with zero thrust
        engine.step(0.05)
        self.assertAlmostEqual(engine.position[2], 0.0)
        self.assertAlmostEqual(engine.velocity[2], 0.0)
        self.assertAlmostEqual(engine.acceleration[2], 0.0)

    def test_ground_constraint_does_not_affect_free_flight(self) -> None:
        drag_coeff = 0.5
        engine = PhysicsEngine(
            position=(0.0, 0.0, 20.0),
            velocity=(0.0, 0.0, 3.0),
            target_thrust_accel=GRAVITY + 2.0,
            actual_thrust_accel=GRAVITY + 2.0,
            drag_coeff_vertical=drag_coeff,
        )
        dt = 0.1
        expected_az = (GRAVITY + 2.0) - GRAVITY - drag_coeff * 3.0
        expected_vz = 3.0 + expected_az * dt
        expected_z = 20.0 + expected_vz * dt

        engine.step(dt)
        self.assertAlmostEqual(engine.acceleration[2], expected_az)
        self.assertAlmostEqual(engine.velocity[2], expected_vz)
        self.assertAlmostEqual(engine.position[2], expected_z)


if __name__ == "__main__":
    unittest.main()
