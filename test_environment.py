import unittest

from environment.model import Environment, GustEvent
from physics import PhysicsEngine
from sensors.barometer import BarometerSensor
from sensors.gps import GPSSensor


class EnvironmentTests(unittest.TestCase):
    def test_steady_wind_acceleration(self):
        env = Environment(steady_wind=(2.0, 0.0), turbulence_strength=0.0, enabled=True, drag_coef=1.0, seed=123)
        engine = PhysicsEngine()
        engine.position = (0.0, 0.0, 0.0)
        engine.velocity = (0.0, 0.0, 0.0)
        # one second step should produce vx ~= wind * drag_coef * dt
        engine.step(1.0, environment=env, sim_time=0.0)
        self.assertAlmostEqual(engine.velocity[0], 2.0 * 1.0 * 1.0, places=5)

    def test_gust_recovery(self):
        g = GustEvent(start=0.0, duration=0.5, strength=(5.0, 0.0))
        env = Environment(steady_wind=(0.0, 0.0), gusts=[g], turbulence_strength=0.0, seed=1)
        # during gust
        wx1, _ = env.get_wind_velocity((0, 0, 0), (0, 0, 0), 0.25)
        # after gust
        wx2, _ = env.get_wind_velocity((0, 0, 0), (0, 0, 0), 1.0)
        self.assertGreater(wx1, wx2)

    def test_turbulence_bounds(self):
        env = Environment(steady_wind=(0.0, 0.0), turbulence_strength=1.0, seed=42)
        for t in [0.0, 0.1, 0.2, 1.0]:
            wx, wy = env.get_wind_velocity((0, 0, 0), (0, 0, 0), t)
            # deviation from steady should be within turbulence_strength + small epsilon
            self.assertLessEqual(abs(wx - 0.0), 1.0 + 1e-6)
            self.assertLessEqual(abs(wy - 0.0), 1.0 + 1e-6)

    def test_seeded_determinism(self):
        env1 = Environment(turbulence_strength=0.5, seed=99)
        env2 = Environment(turbulence_strength=0.5, seed=99)
        # sequences of barometer noise should match
        n1 = [env1.get_barometer_noise() for _ in range(5)]
        n2 = [env2.get_barometer_noise() for _ in range(5)]
        self.assertEqual(n1, n2)

    def test_noisy_sensor_outputs(self):
        env = Environment(turbulence_strength=0.5, seed=7)
        engine = PhysicsEngine(position=(1.0, 2.0, 10.0))
        baro = BarometerSensor()
        gps = GPSSensor(available=True)
        baro.read_from_physics(engine, environment=env)
        gps.read_from_physics(engine, environment=env)
        # barometer altitude should be near 10.0 but not exactly (noise added)
        self.assertNotEqual(baro.altitude, 10.0)
        # gps.last_position should exist and differ from true position when noise enabled
        self.assertNotEqual(getattr(gps, 'last_position', None), engine.position)


if __name__ == "__main__":
    unittest.main()
