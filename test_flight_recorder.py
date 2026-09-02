import io
import os
import unittest

from flight_recorder import FlightRecorder, Replay


class FlightRecorderTests(unittest.TestCase):
    def test_record_and_entries(self):
        r = FlightRecorder()
        r.record_event(0.0, "TEST_EVENT", {"a": 1})
        r.record_step(0.0, {"sim_time": 0.0, "mode": "TEST", "altitude": 1.0})
        entries = list(r.entries())
        self.assertEqual(len(entries), 1)
        self.assertEqual(entries[0].sim_time, 0.0)
        self.assertEqual(len(entries[0].events), 1)

    def test_export_csv_to_stringio(self):
        r = FlightRecorder()
        r.record_step(0.1, {"sim_time": 0.1, "mode": "OK", "altitude": 2.0, "vz": 0.1, "battery": 99.0, "target_altitude": None})
        sio = io.StringIO()
        r.export_csv(sio)
        data = sio.getvalue()
        self.assertIn("sim_time", data)
        self.assertIn("0.100000", data)

    def test_export_csv_to_path(self):
        r = FlightRecorder()
        r.record_step(0.2, {"sim_time": 0.2, "mode": "OK", "altitude": 3.0})
        path = "test_flight_log.csv"
        try:
            r.export_csv(path)
            self.assertTrue(os.path.exists(path))
            with open(path, "r") as f:
                contents = f.read()
                self.assertIn("0.200000", contents)
        finally:
            try:
                os.remove(path)
            except Exception:
                pass

    def test_replay_sequence_and_reset(self):
        r = FlightRecorder()
        r.record_step(0.0, {"sim_time": 0.0, "mode": "A"})
        r.record_step(0.1, {"sim_time": 0.1, "mode": "B"})
        replay = Replay(list(r.entries()))
        seq = [e.sim_time for e in replay]
        self.assertEqual(seq, [0.0, 0.1])
        replay.reset()
        seq2 = [e.sim_time for e in replay]
        self.assertEqual(seq2, [0.0, 0.1])

    def test_empty_log_export(self):
        r = FlightRecorder()
        sio = io.StringIO()
        r.export_csv(sio)
        data = sio.getvalue()
        # header should still be present
        self.assertIn("sim_time", data)


if __name__ == "__main__":
    unittest.main()
