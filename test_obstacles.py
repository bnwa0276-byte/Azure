import unittest
from obstacles.representation import Obstacle
from obstacles.detector import ObstacleDetector
from obstacles.planner import AvoidancePlanner


class ObstacleTests(unittest.TestCase):
    def test_collision_detection_simple(self):
        obs = Obstacle(x=5.0, y=0.0, radius=1.0, z_min=0.0, z_max=10.0)
        detector = ObstacleDetector([obs], drone_radius=0.5, horizon=10.0)
        # drone at origin heading directly to +x at 1 m/s
        pred = detector.predict_collision((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), dt_step=0.5)
        self.assertTrue(pred.will_collide)
        self.assertIsNotNone(pred.obstacle)

    def test_no_collision_when_avoiding(self):
        obs = Obstacle(x=5.0, y=0.0, radius=1.0, z_min=0.0, z_max=10.0)
        detector = ObstacleDetector([obs], drone_radius=0.5, horizon=10.0)
        # heading along y avoids obstacle at x=5
        pred = detector.predict_collision((0.0, 0.0, 1.0), (0.0, 1.0, 0.0), dt_step=0.5)
        self.assertFalse(pred.will_collide)

    def test_avoidance_planner_tangent(self):
        obs = Obstacle(x=1.0, y=0.0, radius=0.5)
        planner = AvoidancePlanner(safety_margin=0.5)
        plan = planner.plan((0.0, 0.0, 1.0), (1.0, 0.0), obs, desired_speed=1.0)
        # should produce a lateral (non-zero vy or vx) to skirt
        self.assertNotEqual(plan.vx, 0.0 or plan.vy)


if __name__ == "__main__":
    unittest.main()
