import unittest

from visualization.world import map_world_to_screen, clamp_viewport
from visualization.telemetry import format_telemetry
from visualization.ui_helpers import telemetry_text_block


class VisualizationHelperTests(unittest.TestCase):
    def test_map_world_to_screen(self):
        p = (1.5, -2.0, 3.0)
        x, y = map_world_to_screen(p)
        self.assertEqual((x, y), (1.5, -2.0))

    def test_clamp_viewport_empty(self):
        bbox = clamp_viewport([])
        self.assertIsInstance(bbox, tuple)

    def test_format_telemetry_and_block(self):
        tel = {'sim_time': 1.23, 'mode': 'HOVER', 'altitude': 5.0, 'vz': 0.1, 'battery': 99.0, 'target_altitude': 10.0}
        f = format_telemetry(tel)
        s = telemetry_text_block(f)
        self.assertIn('sim_time', f)
        self.assertIn('\n', s)


if __name__ == '__main__':
    unittest.main()
