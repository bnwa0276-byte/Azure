import unittest

from drone import (
    Drone,
    FlightMode,
    HealthStatus,
    SensorStatus,
    TransitionError,
)
from flight_controller import FlightController
from sensors.imu import IMUSensor
from hal.simulated import SimulatedSensors
from simulation_runner import SimulationRunner
from environment.model import Environment
from fusion import ComplementaryEstimator


class DroneFSMTests(unittest.TestCase):
    def test_valid_transition_updates_mode(self) -> None:
        drone = Drone()

        drone.change_mode(FlightMode.INITIALIZING)
        drone.change_mode(FlightMode.IDLE)
        drone.change_mode(FlightMode.ARMED)

        self.assertEqual(drone.mode, FlightMode.ARMED)
        self.assertEqual(drone.status, FlightMode.ARMED.value)

    def test_invalid_transition_raises_error(self) -> None:
        drone = Drone()

        with self.assertRaisesRegex(TransitionError, "Invalid transition"):
            drone.change_mode(FlightMode.HOVER)

    def test_transition_history_logs_timestamps(self) -> None:
        drone = Drone()
        drone.change_mode(FlightMode.INITIALIZING)

        self.assertEqual(len(drone.transition_history), 1)
        self.assertIn("timestamp", drone.transition_history[0])
        self.assertIn("from_mode", drone.transition_history[0])
        self.assertIn("to_mode", drone.transition_history[0])

    def test_hover_can_land_directly(self) -> None:
        drone = Drone()
        drone.change_mode(FlightMode.INITIALIZING)
        drone.change_mode(FlightMode.IDLE)
        drone.change_mode(FlightMode.ARMED)
        temp = FlightController(drone)
        drone.takeoff(flight_controller=temp)
        # run simulation until hover
        for _ in range(200):
            temp.update(0.05)
            drone.step_physics(0.05)
            if drone.mode == FlightMode.HOVER:
                break

        self.assertEqual(drone.mode, FlightMode.HOVER)
        # request landing and run controller loop until idle
        fc = FlightController(drone)
        drone.land()
        for _ in range(400):
            fc.update(0.05)
            drone.step_physics(0.05)
            if drone.mode == FlightMode.IDLE:
                break

        # landing sequence is physics-driven and completes to IDLE
        self.assertEqual(drone.mode, FlightMode.IDLE)
        self.assertEqual(drone.altitude, 0.0)

    def test_cannot_arm_with_unhealthy_subsystems(self) -> None:
        health_monitor = HealthStatus(
            battery_level=20.0,
            gps_available=True,
            imu_status=SensorStatus.OK,
            motor_status=SensorStatus.OK,
        )
        drone = Drone(health_monitor=health_monitor)
        drone.change_mode(FlightMode.INITIALIZING)
        drone.change_mode(FlightMode.IDLE)

        with self.assertRaisesRegex(TransitionError, "Cannot arm"):
            drone.change_mode(FlightMode.ARMED)

    def test_emergency_on_critical_failure_while_airborne(self) -> None:
        drone = Drone()
        drone.change_mode(FlightMode.INITIALIZING)
        drone.change_mode(FlightMode.IDLE)
        drone.change_mode(FlightMode.ARMED)
        temp = FlightController(drone)
        drone.takeoff(flight_controller=temp)
        for _ in range(200):
            temp.update(0.05)
            drone.step_physics(0.05)
            if drone.mode == FlightMode.HOVER:
                break

        drone.update_motor_status(SensorStatus.FAILED)

        self.assertEqual(drone.mode, FlightMode.EMERGENCY)


class IMUPipelineTests(unittest.TestCase):
    """Regression tests for ES-024J: IMU data pipeline."""

    def test_imu_initialization_default_accel(self) -> None:
        """Requirement A: IMUSensor last_accel exists immediately with default (0.0, 0.0, 0.0)."""
        imu = IMUSensor()
        self.assertTrue(hasattr(imu, "last_accel"))
        self.assertEqual(imu.last_accel, (0.0, 0.0, 0.0))

    def test_hover_static_sampling(self) -> None:
        """Requirement B: Hover vertical acceleration is 0.0 and reflected in IMU."""
        drone = Drone()
        drone.command_hover()
        drone.step_physics(0.1)
        self.assertAlmostEqual(drone.physics.acceleration[2], 0.0)
        self.assertEqual(drone.health_monitor.imu.last_accel, drone.physics.acceleration)
        self.assertAlmostEqual(drone.health_monitor.imu.last_accel[2], 0.0)

    def test_climb_non_zero_acceleration(self) -> None:
        """Requirement C: Commanded climb produces non-zero acceleration equal to physics.acceleration."""
        drone = Drone()
        drone.command_climb(accel=3.0)
        drone.step_physics(0.1)
        self.assertGreater(drone.physics.acceleration[2], 0.0)
        self.assertEqual(drone.health_monitor.imu.last_accel, drone.physics.acceleration)
        self.assertAlmostEqual(drone.health_monitor.imu.last_accel[2], drone.physics.acceleration[2])

    def test_horizontal_acceleration_from_wind(self) -> None:
        """Requirement D: Wind/drag horizontal acceleration reaches the IMU."""
        env = Environment(steady_wind=(10.0, 0.0), turbulence_strength=0.0, enabled=True, drag_coef=0.5)
        drone = Drone()
        drone.physics.position = (0.0, 0.0, 10.0)
        drone.physics.velocity = (0.0, 0.0, 0.0)
        drone.step_physics(0.1, environment=env)
        # Steady wind accelerates the stationary drone in +x direction
        self.assertGreater(drone.physics.acceleration[0], 0.0)
        self.assertEqual(drone.health_monitor.imu.last_accel, drone.physics.acceleration)
        self.assertAlmostEqual(drone.health_monitor.imu.last_accel[0], drone.physics.acceleration[0])

    def test_ground_contact_zero_vertical_acceleration(self) -> None:
        """Requirement E: Supported ground state reports zero vertical acceleration through IMU."""
        drone = Drone()
        drone.physics.position = (0.0, 0.0, 0.0)
        drone.physics.velocity = (0.0, 0.0, 0.0)
        drone.physics.set_thrust_acceleration(0.0)
        drone.step_physics(0.1)
        # Ground contact normal reaction force balances gravity -> az = 0
        self.assertAlmostEqual(drone.physics.acceleration[2], 0.0)
        self.assertEqual(drone.health_monitor.imu.last_accel, drone.physics.acceleration)
        self.assertAlmostEqual(drone.health_monitor.imu.last_accel[2], 0.0)

    def test_hal_delivery(self) -> None:
        """Requirement F: SimulatedSensors.last_accel() returns the acceleration from IMU."""
        drone = Drone()
        drone.command_climb(accel=4.0)
        drone.step_physics(0.1)
        sensors = SimulatedSensors(drone.health_monitor)
        self.assertEqual(sensors.last_accel(), drone.health_monitor.imu.last_accel)
        self.assertEqual(sensors.last_accel(), drone.physics.acceleration)

    def test_estimator_delivery(self) -> None:
        """Requirement G: SimulationRunner passes valid IMU acceleration (not None) to estimator."""
        class RecordingEstimator(ComplementaryEstimator):
            def __init__(self) -> None:
                super().__init__()
                self.recorded_imu_accel = None

            def update(self, dt, gps_pos, baro_alt, imu_accel, gps_available=True):
                self.recorded_imu_accel = imu_accel
                return super().update(dt, gps_pos, baro_alt, imu_accel, gps_available)

        drone = Drone()
        drone.command_climb(accel=2.0)
        controller = FlightController(drone)
        estimator = RecordingEstimator()
        runner = SimulationRunner(drone, controller, dt=0.05, estimator=estimator)
        runner.step()

        self.assertIsNotNone(estimator.recorded_imu_accel)
        self.assertEqual(estimator.recorded_imu_accel, drone.health_monitor.imu.last_accel)
        self.assertEqual(estimator.recorded_imu_accel, drone.physics.acceleration)


if __name__ == "__main__":
    unittest.main()
