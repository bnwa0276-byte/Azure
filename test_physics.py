import unittest

from physics import PhysicsEngine, GRAVITY
from drone import Drone, FlightMode
from flight_controller import FlightController
from environment.model import Environment
from hal.simulated import SimulatedSensors


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

    def test_command_hover_clamped_at_zero_battery(self) -> None:
        drone = Drone(battery=0.0)
        drone.command_hover()
        self.assertEqual(drone.physics.target_thrust_accel, 0.0)

    def test_command_climb_clamped_at_degraded_battery(self) -> None:
        drone = Drone(battery=15.0)
        requested_thrust = 15.0 + GRAVITY
        self.assertGreater(requested_thrust, drone.available_thrust_limit())
        drone.command_climb(accel=15.0)
        self.assertAlmostEqual(drone.physics.target_thrust_accel, drone.available_thrust_limit())

    def test_command_descend_clamps_excessive_acceleration_to_zero_thrust(self) -> None:
        drone = Drone()
        requested_thrust = GRAVITY - 20.0
        self.assertLess(requested_thrust, 0.0)
        drone.command_descend(accel=20.0)
        self.assertEqual(drone.physics.target_thrust_accel, 0.0)

    def test_flight_controller_commands_respect_battery_limit(self) -> None:
        drone = Drone(battery=0.0)
        fc = FlightController(drone)
        fc.hover()
        self.assertEqual(drone.physics.target_thrust_accel, 0.0)
        fc.climb(3.0)
        self.assertEqual(drone.physics.target_thrust_accel, 0.0)


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


class HorizontalDragTests(unittest.TestCase):
    def test_default_coefficient_preserves_existing_behavior(self) -> None:
        engine_default = PhysicsEngine(position=(1.0, 2.0, 5.0), velocity=(3.0, -4.0, 2.0))
        engine_explicit_zero = PhysicsEngine(
            position=(1.0, 2.0, 5.0),
            velocity=(3.0, -4.0, 2.0),
            drag_coeff_horizontal=0.0,
        )
        self.assertEqual(engine_default.drag_coeff_horizontal, 0.0)
        engine_default.step(0.1)
        engine_explicit_zero.step(0.1)
        self.assertEqual(engine_default.position, engine_explicit_zero.position)
        self.assertEqual(engine_default.velocity, engine_explicit_zero.velocity)
        self.assertEqual(engine_default.acceleration, engine_explicit_zero.acceleration)

    def test_positive_vx_receives_opposing_drag(self) -> None:
        engine_no_drag = PhysicsEngine(position=(0.0, 0.0, 10.0), velocity=(4.0, 0.0, 0.0), drag_coeff_horizontal=0.0)
        engine_with_drag = PhysicsEngine(position=(0.0, 0.0, 10.0), velocity=(4.0, 0.0, 0.0), drag_coeff_horizontal=0.5)

        engine_no_drag.step(0.1)
        engine_with_drag.step(0.1)

        # ax without drag: 0.0; ax with drag: -0.5 * 4.0 = -2.0
        self.assertAlmostEqual(engine_no_drag.acceleration[0], 0.0)
        self.assertAlmostEqual(engine_with_drag.acceleration[0], -2.0)
        self.assertLess(engine_with_drag.acceleration[0], 0.0)
        self.assertLess(engine_with_drag.velocity[0], engine_no_drag.velocity[0])
        self.assertLess(engine_with_drag.position[0], engine_no_drag.position[0])

    def test_negative_vx_receives_opposing_drag(self) -> None:
        engine_no_drag = PhysicsEngine(position=(0.0, 0.0, 10.0), velocity=(-4.0, 0.0, 0.0), drag_coeff_horizontal=0.0)
        engine_with_drag = PhysicsEngine(position=(0.0, 0.0, 10.0), velocity=(-4.0, 0.0, 0.0), drag_coeff_horizontal=0.5)

        engine_no_drag.step(0.1)
        engine_with_drag.step(0.1)

        # ax without drag: 0.0; ax with drag: -0.5 * (-4.0) = +2.0
        self.assertAlmostEqual(engine_no_drag.acceleration[0], 0.0)
        self.assertAlmostEqual(engine_with_drag.acceleration[0], 2.0)
        self.assertGreater(engine_with_drag.acceleration[0], 0.0)
        self.assertGreater(engine_with_drag.velocity[0], engine_no_drag.velocity[0])
        self.assertGreater(engine_with_drag.position[0], engine_no_drag.position[0])

    def test_positive_vy_receives_opposing_drag(self) -> None:
        engine_no_drag = PhysicsEngine(position=(0.0, 0.0, 10.0), velocity=(0.0, 4.0, 0.0), drag_coeff_horizontal=0.0)
        engine_with_drag = PhysicsEngine(position=(0.0, 0.0, 10.0), velocity=(0.0, 4.0, 0.0), drag_coeff_horizontal=0.5)

        engine_no_drag.step(0.1)
        engine_with_drag.step(0.1)

        # ay without drag: 0.0; ay with drag: -0.5 * 4.0 = -2.0
        self.assertAlmostEqual(engine_no_drag.acceleration[1], 0.0)
        self.assertAlmostEqual(engine_with_drag.acceleration[1], -2.0)
        self.assertLess(engine_with_drag.acceleration[1], 0.0)
        self.assertLess(engine_with_drag.velocity[1], engine_no_drag.velocity[1])
        self.assertLess(engine_with_drag.position[1], engine_no_drag.position[1])

    def test_negative_vy_receives_opposing_drag(self) -> None:
        engine_no_drag = PhysicsEngine(position=(0.0, 0.0, 10.0), velocity=(0.0, -4.0, 0.0), drag_coeff_horizontal=0.0)
        engine_with_drag = PhysicsEngine(position=(0.0, 0.0, 10.0), velocity=(0.0, -4.0, 0.0), drag_coeff_horizontal=0.5)

        engine_no_drag.step(0.1)
        engine_with_drag.step(0.1)

        # ay without drag: 0.0; ay with drag: -0.5 * (-4.0) = +2.0
        self.assertAlmostEqual(engine_no_drag.acceleration[1], 0.0)
        self.assertAlmostEqual(engine_with_drag.acceleration[1], 2.0)
        self.assertGreater(engine_with_drag.acceleration[1], 0.0)
        self.assertGreater(engine_with_drag.velocity[1], engine_no_drag.velocity[1])
        self.assertGreater(engine_with_drag.position[1], engine_no_drag.position[1])

    def test_zero_horizontal_velocity_produces_zero_drag(self) -> None:
        engine = PhysicsEngine(position=(5.0, 5.0, 10.0), velocity=(0.0, 0.0, 0.0), drag_coeff_horizontal=1.5)
        engine.step(0.1)
        self.assertAlmostEqual(engine.acceleration[0], 0.0)
        self.assertAlmostEqual(engine.acceleration[1], 0.0)
        self.assertAlmostEqual(engine.velocity[0], 0.0)
        self.assertAlmostEqual(engine.velocity[1], 0.0)
        self.assertAlmostEqual(engine.position[0], 5.0)
        self.assertAlmostEqual(engine.position[1], 5.0)

    def test_negative_drag_coefficient_raises_value_error(self) -> None:
        with self.assertRaises(ValueError):
            PhysicsEngine(drag_coeff_horizontal=-0.5)
        with self.assertRaises(ValueError):
            PhysicsEngine(drag_coeff_horizontal=-1e-5)

    def test_horizontal_and_vertical_drag_operate_independently(self) -> None:
        drag_h = 0.4
        drag_v = 0.8
        vx0, vy0, vz0 = 3.0, -5.0, 2.0
        dt = 0.1

        engine = PhysicsEngine(
            position=(10.0, 20.0, 30.0),
            velocity=(vx0, vy0, vz0),
            drag_coeff_horizontal=drag_h,
            drag_coeff_vertical=drag_v,
        )

        expected_ax = -drag_h * vx0
        expected_ay = -drag_h * vy0
        expected_az = -drag_v * vz0
        expected_vx = vx0 + expected_ax * dt
        expected_vy = vy0 + expected_ay * dt
        expected_vz = vz0 + expected_az * dt
        expected_x = 10.0 + expected_vx * dt
        expected_y = 20.0 + expected_vy * dt
        expected_z = 30.0 + expected_vz * dt

        engine.step(dt)

        self.assertAlmostEqual(engine.acceleration[0], expected_ax)
        self.assertAlmostEqual(engine.acceleration[1], expected_ay)
        self.assertAlmostEqual(engine.acceleration[2], expected_az)
        self.assertAlmostEqual(engine.velocity[0], expected_vx)
        self.assertAlmostEqual(engine.velocity[1], expected_vy)
        self.assertAlmostEqual(engine.velocity[2], expected_vz)
        self.assertAlmostEqual(engine.position[0], expected_x)
        self.assertAlmostEqual(engine.position[1], expected_y)
        self.assertAlmostEqual(engine.position[2], expected_z)

        engine_h_only = PhysicsEngine(
            position=(10.0, 20.0, 30.0),
            velocity=(vx0, vy0, vz0),
            drag_coeff_horizontal=drag_h,
            drag_coeff_vertical=0.0,
        )
        engine_h_only.step(dt)
        self.assertAlmostEqual(engine.acceleration[0], engine_h_only.acceleration[0])
        self.assertAlmostEqual(engine.acceleration[1], engine_h_only.acceleration[1])
        self.assertAlmostEqual(engine.velocity[0], engine_h_only.velocity[0])
        self.assertAlmostEqual(engine.velocity[1], engine_h_only.velocity[1])
        self.assertNotEqual(engine.acceleration[2], engine_h_only.acceleration[2])


class RelativeWindAerodynamicDragTests(unittest.TestCase):
    """Regression tests for ES-024I: unified aerodynamic drag and relative wind."""

    def test_zero_relative_wind_produces_zero_drag(self) -> None:
        """Requirement A: vehicle velocity = (5, -3, 0), wind = (5, -3) -> drag = (0, 0)."""
        engine = PhysicsEngine(
            position=(0.0, 0.0, 10.0),
            velocity=(5.0, -3.0, 0.0),
            drag_coeff_horizontal=0.5,
        )
        env = Environment(steady_wind=(5.0, -3.0), turbulence_strength=0.0, enabled=True)
        engine.step(0.1, environment=env)

        self.assertAlmostEqual(engine.acceleration[0], 0.0)
        self.assertAlmostEqual(engine.acceleration[1], 0.0)
        self.assertAlmostEqual(engine.velocity[0], 5.0)
        self.assertAlmostEqual(engine.velocity[1], -3.0)

    def test_still_air_drag_opposes_velocity(self) -> None:
        """Requirement B: wind = (0, 0), velocity = (5, -3, 0) -> drag opposes velocity."""
        engine = PhysicsEngine(
            position=(0.0, 0.0, 10.0),
            velocity=(5.0, -3.0, 0.0),
            drag_coeff_horizontal=0.5,
        )
        env = Environment(steady_wind=(0.0, 0.0), turbulence_strength=0.0, enabled=True)
        engine.step(0.1, environment=env)

        # ax = -0.5 * 5.0 = -2.5; ay = -0.5 * (-3.0) = +1.5
        self.assertAlmostEqual(engine.acceleration[0], -2.5)
        self.assertAlmostEqual(engine.acceleration[1], 1.5)
        self.assertLess(engine.velocity[0], 5.0)
        self.assertGreater(engine.velocity[1], -3.0)

    def test_headwind_doubles_drag_relative_to_still_air(self) -> None:
        """Requirement C: vehicle +5 m/s, wind -5 m/s -> relative airspeed +10 m/s.

        Drag magnitude must be twice that of the still-air (+5 m/s vehicle, 0 m/s wind) case.
        """
        engine_still = PhysicsEngine(
            position=(0.0, 0.0, 10.0),
            velocity=(5.0, 0.0, 0.0),
            drag_coeff_horizontal=0.5,
        )
        env_still = Environment(steady_wind=(0.0, 0.0), turbulence_strength=0.0, enabled=True)
        engine_still.step(0.1, environment=env_still)

        engine_headwind = PhysicsEngine(
            position=(0.0, 0.0, 10.0),
            velocity=(5.0, 0.0, 0.0),
            drag_coeff_horizontal=0.5,
        )
        env_headwind = Environment(steady_wind=(-5.0, 0.0), turbulence_strength=0.0, enabled=True)
        engine_headwind.step(0.1, environment=env_headwind)

        # Still: a = -0.5 * 5.0 = -2.5; Headwind: a = -0.5 * (5.0 - (-5.0)) = -5.0
        self.assertAlmostEqual(engine_still.acceleration[0], -2.5)
        self.assertAlmostEqual(engine_headwind.acceleration[0], -5.0)
        self.assertAlmostEqual(abs(engine_headwind.acceleration[0]), 2.0 * abs(engine_still.acceleration[0]))

    def test_tailwind_matching_velocity_produces_zero_drag(self) -> None:
        """Requirement D: vehicle velocity = wind velocity -> horizontal drag is zero."""
        engine = PhysicsEngine(
            position=(0.0, 0.0, 10.0),
            velocity=(8.0, 4.0, 0.0),
            drag_coeff_horizontal=0.8,
        )
        env = Environment(steady_wind=(8.0, 4.0), turbulence_strength=0.0, enabled=True)
        engine.step(0.1, environment=env)

        self.assertAlmostEqual(engine.acceleration[0], 0.0)
        self.assertAlmostEqual(engine.acceleration[1], 0.0)
        self.assertAlmostEqual(engine.velocity[0], 8.0)
        self.assertAlmostEqual(engine.velocity[1], 4.0)

    def test_no_double_counting_with_environment_and_physics(self) -> None:
        """Requirement E: canonical Cd = 0.5, vx = 4 m/s in still air -> ax = -2.0 m/s^2, NOT -4.0 m/s^2."""
        engine = PhysicsEngine(
            position=(0.0, 0.0, 10.0),
            velocity=(4.0, 0.0, 0.0),
            drag_coeff_horizontal=0.5,
        )
        env = Environment(steady_wind=(0.0, 0.0), turbulence_strength=0.0, drag_coef=0.5, enabled=True)
        engine.step(0.1, environment=env)

        # Aerodynamic drag must be applied exactly once (-0.5 * 4.0 = -2.0 m/s^2)
        self.assertAlmostEqual(engine.acceleration[0], -2.0)
        self.assertNotEqual(engine.acceleration[0], -4.0)

    def test_direct_wind_velocity_parameter(self) -> None:
        """Requirement F: PhysicsEngine accepts direct wind_velocity without Environment object."""
        engine = PhysicsEngine(
            position=(0.0, 0.0, 10.0),
            velocity=(5.0, -3.0, 0.0),
            drag_coeff_horizontal=0.5,
        )
        # zero relative wind directly
        engine.step(0.1, wind_velocity=(5.0, -3.0))
        self.assertAlmostEqual(engine.acceleration[0], 0.0)
        self.assertAlmostEqual(engine.acceleration[1], 0.0)

        # opposing wind directly: v_rel = 5.0 - (-5.0) = 10.0 -> a = -5.0
        engine.step(0.1, wind_velocity=(-5.0, -3.0))
        self.assertAlmostEqual(engine.acceleration[0], -5.0)
        self.assertAlmostEqual(engine.acceleration[1], 0.0)


class ImplicitAerodynamicDragTests(unittest.TestCase):
    """Regression tests for ES-024K: opt-in implicit aerodynamic drag integration."""

    def test_opt_in_flag(self) -> None:
        """Requirement A: Default engine uses explicit mode; implicit_drag=True activates implicit mode."""
        engine_default = PhysicsEngine()
        self.assertFalse(engine_default.implicit_drag)

        engine_implicit = PhysicsEngine(implicit_drag=True)
        self.assertTrue(engine_implicit.implicit_drag)

    def test_large_cd_dt_stability(self) -> None:
        """Requirement B: Deliberately coarse timesteps (Cd*dt = 5.0, 10.0) do not oscillate or diverge."""
        # Cd = 2.0, dt = 2.5 -> Cd*dt = 5.0 (explicit Euler would amplify perturbation by 1 - 5 = -4, diverging!)
        engine = PhysicsEngine(
            position=(0.0, 0.0, 10.0),
            velocity=(12.0, -6.0, 0.0),
            drag_coeff_horizontal=2.0,
            implicit_drag=True,
        )
        engine.step(2.5)

        # In implicit Euler, v_new = v_old / (1 + 5.0) = v_old / 6.0
        expected_vx = 12.0 / 6.0  # 2.0
        expected_vy = -6.0 / 6.0  # -1.0
        self.assertAlmostEqual(engine.velocity[0], expected_vx, places=5)
        self.assertAlmostEqual(engine.velocity[1], expected_vy, places=5)
        self.assertGreater(engine.velocity[0], 0.0)
        self.assertLess(engine.velocity[1], 0.0)

        # Next step with dt = 5.0 -> Cd*dt = 10.0
        engine.step(5.0)
        expected_vx_step2 = expected_vx / (1.0 + 2.0 * 5.0)  # 2.0 / 11.0
        expected_vy_step2 = expected_vy / (1.0 + 2.0 * 5.0)  # -1.0 / 11.0
        self.assertAlmostEqual(engine.velocity[0], expected_vx_step2, places=5)
        self.assertAlmostEqual(engine.velocity[1], expected_vy_step2, places=5)
        self.assertGreater(engine.velocity[0], 0.0)
        self.assertLess(engine.velocity[1], 0.0)

    def test_monotonic_convergence_in_still_air(self) -> None:
        """Requirement C: In still air, velocity damps toward zero monotonically without sign-flipping."""
        # Cd = 1.5, dt = 1.0 -> Cd*dt = 1.5 (in explicit Euler, 1 < Cd*dt < 2 causes sign-flipping oscillation)
        engine = PhysicsEngine(
            position=(0.0, 0.0, 10.0),
            velocity=(20.0, 0.0, 0.0),
            drag_coeff_horizontal=1.5,
            implicit_drag=True,
        )

        for _ in range(10):
            prev_vx = engine.velocity[0]
            engine.step(1.0)
            curr_vx = engine.velocity[0]
            # Must strictly decrease in magnitude and never flip sign
            self.assertGreater(curr_vx, 0.0)
            self.assertLess(curr_vx, prev_vx)

        self.assertAlmostEqual(engine.velocity[0], 0.0, delta=0.01)

    def test_wind_convergence_without_oscillation(self) -> None:
        """Requirement D: With nonzero wind, velocity approaches wind without oscillatory sign-flipping."""
        # Wind = (10.0, 0.0), Cd = 2.0, dt = 1.0 -> Cd*dt = 2.0 (marginal stability in explicit Euler)
        engine = PhysicsEngine(
            position=(0.0, 0.0, 10.0),
            velocity=(0.0, 0.0, 0.0),
            drag_coeff_horizontal=2.0,
            implicit_drag=True,
        )
        env = Environment(steady_wind=(10.0, 0.0), turbulence_strength=0.0, enabled=True)

        for _ in range(8):
            prev_vx = engine.velocity[0]
            engine.step(1.0, environment=env)
            curr_vx = engine.velocity[0]
            # Velocity monotonically approaches 10.0 from below, never overshooting 10.0
            self.assertGreaterEqual(curr_vx, prev_vx)
            self.assertLessEqual(curr_vx, 10.0)

        self.assertAlmostEqual(engine.velocity[0], 10.0, delta=0.05)

    def test_zero_relative_wind(self) -> None:
        """Requirement E: Vehicle velocity = wind velocity -> zero drag acceleration."""
        engine = PhysicsEngine(
            position=(0.0, 0.0, 10.0),
            velocity=(6.0, -3.0, 0.0),
            drag_coeff_horizontal=0.5,
            implicit_drag=True,
        )
        env = Environment(steady_wind=(6.0, -3.0), turbulence_strength=0.0, enabled=True)
        engine.step(0.1, environment=env)

        self.assertAlmostEqual(engine.acceleration[0], 0.0)
        self.assertAlmostEqual(engine.acceleration[1], 0.0)
        self.assertAlmostEqual(engine.velocity[0], 6.0)
        self.assertAlmostEqual(engine.velocity[1], -3.0)

    def test_zero_drag_equivalence(self) -> None:
        """Requirement F: With Cd=0, implicit mode produces identical result to explicit Euler."""
        engine_exp = PhysicsEngine(position=(0.0, 0.0, 10.0), velocity=(5.0, -2.0, 3.0), drag_coeff_horizontal=0.0, drag_coeff_vertical=0.0, implicit_drag=False)
        engine_imp = PhysicsEngine(position=(0.0, 0.0, 10.0), velocity=(5.0, -2.0, 3.0), drag_coeff_horizontal=0.0, drag_coeff_vertical=0.0, implicit_drag=True)

        engine_exp.step(0.05)
        engine_imp.step(0.05)

        self.assertEqual(engine_exp.position, engine_imp.position)
        self.assertEqual(engine_exp.velocity, engine_imp.velocity)
        self.assertEqual(engine_exp.acceleration, engine_imp.acceleration)

    def test_small_step_closeness_to_explicit(self) -> None:
        """Requirement G: With small dt, implicit and explicit solutions agree closely within numerical tolerance."""
        dt = 0.001
        engine_exp = PhysicsEngine(velocity=(5.0, -3.0, 0.0), drag_coeff_horizontal=0.5, implicit_drag=False)
        engine_imp = PhysicsEngine(velocity=(5.0, -3.0, 0.0), drag_coeff_horizontal=0.5, implicit_drag=True)

        engine_exp.step(dt)
        engine_imp.step(dt)

        # O(dt^2) difference: for dt=0.001, Cd=0.5, (Cd*dt)^2 = 2.5e-7
        self.assertAlmostEqual(engine_imp.velocity[0], engine_exp.velocity[0], places=4)
        self.assertAlmostEqual(engine_imp.velocity[1], engine_exp.velocity[1], places=4)

    def test_vertical_drag_large_cd_dt_stability(self) -> None:
        """Requirement H: Implicit vertical drag produces stable non-divergent behavior for large Cd*dt."""
        # Hover thrust so a_non_drag_z = 0, initial vz = 10.0, Cd_v = 3.0, dt = 2.0 -> Cd_v * dt = 6.0
        engine = PhysicsEngine(
            position=(0.0, 0.0, 50.0),
            velocity=(0.0, 0.0, 10.0),
            target_thrust_accel=GRAVITY,
            actual_thrust_accel=GRAVITY,
            drag_coeff_vertical=3.0,
            implicit_drag=True,
        )
        engine.step(2.0)

        # vz_new = 10.0 / (1 + 6.0) = 1.42857...
        expected_vz = 10.0 / 7.0
        self.assertAlmostEqual(engine.velocity[2], expected_vz, places=5)
        self.assertGreater(engine.velocity[2], 0.0)
        self.assertLess(engine.velocity[2], 10.0)

    def test_ground_contact_under_implicit_integration(self) -> None:
        """Requirement I: Implicit integration preserves no-penetration, downward velocity removal, and liftoff."""
        # 1. Contact impact
        engine = PhysicsEngine(
            position=(0.0, 0.0, 0.5),
            velocity=(0.0, 0.0, -5.0),
            target_thrust_accel=0.0,
            actual_thrust_accel=0.0,
            drag_coeff_vertical=0.5,
            implicit_drag=True,
        )
        # Step into ground
        engine.step(0.2)
        self.assertAlmostEqual(engine.position[2], 0.0)
        self.assertAlmostEqual(engine.velocity[2], 0.0)
        self.assertAlmostEqual(engine.acceleration[2], 0.0)

        # 2. Resting contact stability
        engine.step(0.1)
        self.assertAlmostEqual(engine.position[2], 0.0)
        self.assertAlmostEqual(engine.velocity[2], 0.0)
        self.assertAlmostEqual(engine.acceleration[2], 0.0)

        # 3. Liftoff
        climb_thrust = GRAVITY + 4.0
        engine.set_thrust_acceleration(climb_thrust)
        engine.step(0.1)
        self.assertGreater(engine.acceleration[2], 0.0)
        self.assertGreater(engine.velocity[2], 0.0)
        self.assertGreater(engine.position[2], 0.0)

    def test_acceleration_consistency(self) -> None:
        """Requirement J: Reported acceleration equals (velocity_new - velocity_old) / dt."""
        dt = 0.05
        engine = PhysicsEngine(
            position=(0.0, 0.0, 20.0),
            velocity=(4.0, -3.0, 2.0),
            drag_coeff_horizontal=0.6,
            drag_coeff_vertical=0.4,
            implicit_drag=True,
        )
        vx0, vy0, vz0 = engine.velocity
        engine.step(dt)
        vx1, vy1, vz1 = engine.velocity

        self.assertAlmostEqual(engine.acceleration[0], (vx1 - vx0) / dt, places=6)
        self.assertAlmostEqual(engine.acceleration[1], (vy1 - vy0) / dt, places=6)
        self.assertAlmostEqual(engine.acceleration[2], (vz1 - vz0) / dt, places=6)

    def test_imu_compatibility(self) -> None:
        """Requirement K: Acceleration from implicit mode is sampled by IMU and matches physics."""
        drone = Drone()
        drone.physics.implicit_drag = True
        drone.physics.drag_coeff_horizontal = 0.5
        drone.command_climb(accel=3.0)
        drone.step_physics(0.1)

        # IMU should sample the acceleration from physics
        self.assertEqual(drone.health_monitor.imu.last_accel, drone.physics.acceleration)
        self.assertGreater(drone.health_monitor.imu.last_accel[2], 0.0)

        # HAL SimulatedSensors should also reflect this
        sensors = SimulatedSensors(drone.health_monitor)
        self.assertEqual(sensors.last_accel(), drone.physics.acceleration)


if __name__ == "__main__":
    unittest.main()
