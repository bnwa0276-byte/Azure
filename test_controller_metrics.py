import unittest

from metrics.controller_metrics import ControllerMetrics


class ControllerMetricsTests(unittest.TestCase):
    def test_metrics_computation(self) -> None:
        m = ControllerMetrics()
        # simulate a step response: target=10, altitude overshoots then settles
        times = [i * 0.5 for i in range(20)]
        alts = [0.0, 2.0, 6.0, 11.0, 12.0, 11.0, 10.5, 10.2, 10.05, 9.98, 10.0, 10.0]
        # pad times to match alt length
        times = times[: len(alts)]
        for t, a in zip(times, alts):
            m.sample(t, a, 10.0)

        self.assertAlmostEqual(m.max_overshoot(), 2.0, places=3)
        avg_err = m.average_abs_error()
        self.assertGreater(avg_err, 0.0)
        st = m.settling_time(tolerance=0.2, settle_window=0.5)
        self.assertIsNotNone(st)

    def test_recovery_time_after_disturbance(self) -> None:
        m = ControllerMetrics()
        # before disturbance, at target
        for i in range(4):
            m.sample(i * 0.1, 10.0, 10.0)
        # disturbance at t=0.4
        m.sample(0.4, 6.0, 10.0)
        # recovery samples
        m.sample(0.5, 8.0, 10.0)
        m.sample(0.6, 9.5, 10.0)
        m.sample(0.7, 10.1, 10.0)

        rt = m.recovery_time_after(0.4, tolerance=0.2)
        self.assertAlmostEqual(rt, 0.3, places=2)


if __name__ == "__main__":
    unittest.main()
