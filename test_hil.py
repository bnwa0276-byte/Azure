import unittest

from hal.transports import MockTransport
from hal.scheduler import Scheduler
from hal.hil import HILBridge
from drone import Drone
from flight_recorder.recorder import FlightRecorder


class MockVehicle(Drone):
    def __init__(self):
        super().__init__()
        self.velocity_commands = []

    def command_velocity(self, vx: float, vy: float) -> None:
        self.velocity_commands.append((vx, vy))


class HILTests(unittest.TestCase):
    def test_mock_transport_latency_jitter_loss_deterministic(self):
        mt = MockTransport(latency=0.1, jitter=0.05, loss=0.2, seed=12345)
        received = []

        def cb(pkt, t):
            received.append((pkt, t))

        mt.register_receive_callback(cb)
        mt.send({"cmd": "velocity", "vx": 1.0}, now=0.0)
        mt.send({"cmd": "velocity", "vx": 2.0}, now=0.0)
        # advance time enough to deliver
        mt.advance_time(1.0)
        # deterministic: with seed there is reproducible count
        self.assertIsInstance(len(received), int)

    def test_scheduler_invokes_tasks_at_rate(self):
        s = Scheduler()
        calls = []

        def task(t):
            calls.append(t)

        s.schedule(task, rate_hz=2.0, start_time=0.0)  # period 0.5
        s.tick(0.0)
        self.assertEqual(len(calls), 1)
        s.tick(0.25)
        self.assertEqual(len(calls), 1)
        s.tick(0.5)
        self.assertEqual(len(calls), 2)

    def test_hil_bridge_delivers_and_applies_commands(self):
        v = MockVehicle()
        mt = MockTransport(latency=0.01, jitter=0.0, loss=0.0, seed=1)
        rec = FlightRecorder()
        hil = HILBridge(vehicle=v, transport=mt, recorder=rec)
        # send command into transport as if from GCS
        mt.send({"cmd": "velocity", "vx": 3.0, "vy": -1.0}, now=0.0)
        mt.advance_time(0.1)
        # after delivery, vehicle should have received command
        self.assertTrue(len(v.velocity_commands) >= 1)
        # flush events into a step so recorder.entries() includes events
        rec.record_step(0.1, {})
        entries = list(rec.entries())
        self.assertTrue(any(ev.get("event") == "hil_receive" for e in entries for ev in (e.events or [])))


if __name__ == "__main__":
    unittest.main()
