import unittest

from navigation import Mission, NavigationSystem, Waypoint


class NavigationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.waypoints = [
            Waypoint("Home", 0.0, 0.0, 0.0),
            Waypoint("PointA", 0.001, 0.0, 10.0),
            Waypoint("PointB", 0.001, 0.001, 15.0),
        ]
        self.mission = Mission(self.waypoints)
        self.navigation = NavigationSystem(self.mission)

    def test_active_waypoint_progression(self) -> None:
        self.assertEqual(self.navigation.active_waypoint, self.waypoints[0])
        self.navigation.update_position(0.0, 0.0, 0.0)
        self.assertEqual(self.navigation.active_waypoint, self.waypoints[1])
        self.navigation.update_position(0.001, 0.0, 10.0)
        self.assertEqual(self.navigation.active_waypoint, self.waypoints[2])

    def test_mission_completion(self) -> None:
        self.navigation.update_position(0.0, 0.0, 0.0)
        self.navigation.update_position(0.001, 0.0, 10.0)
        status = self.navigation.update_position(0.001, 0.001, 15.0)
        self.assertEqual(status, "MISSION_COMPLETE")
        self.assertTrue(self.mission.is_complete)

    def test_distance_calculation(self) -> None:
        # Use a point away from the first waypoint so the distance is non-zero.
        distance = self.navigation.distance_to_active_waypoint(0.0, -0.001, 0.0)
        self.assertGreater(distance, 0.0)
        self.assertAlmostEqual(distance, 111.0, delta=5.0)

    def test_mission_status_report(self) -> None:
        self.assertEqual(self.navigation.mission_status(), "0/3 waypoints completed")
        self.navigation.update_position(0.0, 0.0, 0.0)
        self.assertEqual(self.navigation.mission_status(), "1/3 waypoints completed")


class NavigationIntegrationTests(unittest.TestCase):
    def test_navigation_integration_with_flight_controller(self) -> None:
        from drone import Drone, FlightMode
        from flight_controller import FlightController

        waypoints = [
            Waypoint("Home", 0.0, 0.0, 0.0),
            Waypoint("PointA", 0.001, 0.0, 10.0),
        ]
        mission = Mission(waypoints)
        navigation = NavigationSystem(mission)

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

        controller = FlightController(drone, navigation)
        status = controller.supervise_mission(0.0, 0.0, 0.0)

        self.assertEqual(status, "WAYPOINT_REACHED")
        self.assertEqual(navigation.active_waypoint, waypoints[1])

        status = controller.supervise_mission(0.001, 0.0, 10.0)
        self.assertEqual(status, "MISSION_COMPLETE")
        self.assertTrue(mission.is_complete)


if __name__ == "__main__":
    unittest.main()
