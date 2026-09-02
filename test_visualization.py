import unittest

from visualization.renderer import MatplotlibVisualizer


class VisualizationTests(unittest.TestCase):
    def test_coordinate_mapping_and_trail(self):
        v = MatplotlibVisualizer()
        # simulate positions
        positions = [(0.0, 0.0, 0.0), (1.0, 0.5, 0.0), (2.0, 1.0, 0.0)]
        telemetry = {'sim_time': 0.0, 'mode': 'ARMED', 'altitude': 0.0, 'vz': 0.0, 'battery': 100.0}
        for p in positions:
            v.update(telemetry=telemetry, position=p, waypoints=[], path=[p])

        # path_history should map to 2D points
        self.assertEqual(len(v.path_history), 3)
        self.assertEqual(v.path_history[0], (0.0, 0.0))

    def test_telemetry_formatting(self):
        v = MatplotlibVisualizer()
        tel = {'sim_time': 1.2345, 'mode': 'HOVER', 'altitude': 3.21, 'vz': -0.5, 'battery': 88.8, 'target_altitude': 5.0}
        fmt = v.format_telemetry(tel)
        self.assertIn('sim_time', fmt)
        self.assertIn('altitude', fmt)
        self.assertIn('battery', fmt)


if __name__ == '__main__':
    unittest.main()
