import unittest

from navigation import Mission, NavigationSystem, Waypoint
from guidance import GuidanceSystem, GuidanceCommand
from drone import Drone
from flight_controller import FlightController


class GuidanceTests(unittest.TestCase):
    def setUp(self) -> None:
        self.waypoints = [
            Waypoint("Home", 0.0, 0.0, 0.0),
            Waypoint("PointA", 0.001, 0.0, 0.0),
        ]
        self.mission = Mission(self.waypoints)
        self.navigation = NavigationSystem(self.mission)
        self.guidance = GuidanceSystem(self.navigation, desired_speed=2.0, acceptance_radius_m=5.0)

    def test_guidance_command_generation(self):
        # advance mission past Home so active waypoint is PointA
        self.navigation.update_position(0.0, 0.0, 0.0)
        # starting at origin, should compute non-zero vx toward PointA
        cmd = self.guidance.compute_command(0.0, 0.0, 0.0)
        self.assertIsInstance(cmd, GuidanceCommand)
        self.assertNotEqual(cmd.vx, 0.0)
        self.assertAlmostEqual(cmd.desired_speed, 2.0)

    def test_waypoint_acquisition_within_radius(self):
        # place drone inside acceptance radius -> should trigger waypoint advancement
        # convert small degrees to meters: 0.0, 0.0 compared to Home (0.0,0.0) is zero
        cmd = self.guidance.compute_command(0.0, 0.0, 0.0)
        # Because active waypoint is Home at same coords, compute_command should advance
        self.assertEqual(self.navigation.active_waypoint, self.waypoints[1])

    def test_flightcontroller_applies_guidance(self):
        drone = Drone()
        fc = FlightController(drone)
        # compute guidance toward PointA from origin
        cmd = self.guidance.compute_command(0.0, 0.0, 0.0)
        # ensure second waypoint is active now
        self.assertEqual(self.navigation.active_waypoint, self.waypoints[1])
        # compute guidance toward PointA from slightly away
        cmd2 = self.guidance.compute_command(0.0, -0.0005, 0.0)
        fc.apply_guidance(cmd2)
        # drone horizontal velocity should be non-zero
        vx, vy, vz = drone.physics.velocity
        self.assertNotEqual(vx, 0.0)


if __name__ == "__main__":
    unittest.main()
