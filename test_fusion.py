import unittest
import random

from fusion import ComplementaryEstimator


class FusionTests(unittest.TestCase):
    def test_deterministic_with_seed(self):
        seed = 42
        rng1 = random.Random(seed)
        rng2 = random.Random(seed)
        est1 = ComplementaryEstimator(alpha=0.95)
        est2 = ComplementaryEstimator(alpha=0.95)
        # produce identical noisy measurements sequences
        for i in range(10):
            dt = 0.1
            # simulate IMU acceleration noise
            imu = (rng1.uniform(-0.1, 0.1), rng1.uniform(-0.1, 0.1), rng1.uniform(-0.1, 0.1))
            imu2 = (rng2.uniform(-0.1, 0.1), rng2.uniform(-0.1, 0.1), rng2.uniform(-0.1, 0.1))
            gps = (rng1.uniform(0.0, 1.0), rng1.uniform(0.0, 1.0), rng1.uniform(0.0, 1.0))
            gps2 = (rng2.uniform(0.0, 1.0), rng2.uniform(0.0, 1.0), rng2.uniform(0.0, 1.0))
            baro = rng1.uniform(0.0, 1.0)
            baro2 = rng2.uniform(0.0, 1.0)
            s1 = est1.update(dt, gps_pos=gps, baro_alt=baro, imu_accel=imu, gps_available=True)
            s2 = est2.update(dt, gps_pos=gps2, baro_alt=baro2, imu_accel=imu2, gps_available=True)
            self.assertAlmostEqual(s1.position[0], s2.position[0], places=6)
            self.assertAlmostEqual(s1.position[2], s2.position[2], places=6)

    def test_temporary_gps_loss_confidence_drop_and_recovery(self):
        est = ComplementaryEstimator(alpha=0.9)
        dt = 0.1
        # initial with GPS
        s = est.update(dt, gps_pos=(0.0, 0.0, 0.0), baro_alt=0.0, imu_accel=(0.0, 0.0, 0.0), gps_available=True)
        conf1 = s.confidence
        # simulate GPS loss
        s2 = est.update(dt, gps_pos=None, baro_alt=0.0, imu_accel=(0.0, 0.0, 0.0), gps_available=False)
        self.assertLessEqual(s2.confidence, conf1)
        # GPS returns
        s3 = est.update(dt, gps_pos=(0.1, 0.0, 0.0), baro_alt=0.0, imu_accel=(0.0, 0.0, 0.0), gps_available=True)
        self.assertGreaterEqual(s3.confidence, s2.confidence)

    def test_estimator_converges_with_noisy_inputs(self):
        import random
        rng = random.Random(1)
        est = ComplementaryEstimator(alpha=0.9)
        state = None
        for i in range(50):
            dt = 0.1
            imu = (rng.uniform(-0.2, 0.2), rng.uniform(-0.2, 0.2), rng.uniform(-0.2, 0.2))
            gps = (i * 0.1 + rng.uniform(-0.05, 0.05), rng.uniform(-0.05, 0.05), 1.0 + rng.uniform(-0.02, 0.02))
            baro = 1.0 + rng.uniform(-0.02, 0.02)
            state = est.update(dt, gps_pos=gps, baro_alt=baro, imu_accel=imu, gps_available=True)

        # After running, estimator position should be near last GPS
        self.assertAlmostEqual(state.position[0], gps[0], delta=0.2)
        self.assertAlmostEqual(state.position[2], gps[2], delta=0.2)


if __name__ == "__main__":
    unittest.main()
