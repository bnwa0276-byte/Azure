import unittest

from drone import (
    Drone,
    FlightMode,
    HealthStatus,
    SensorStatus,
    TransitionError,
)
from flight_controller import FlightController


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


if __name__ == "__main__":
    unittest.main()
