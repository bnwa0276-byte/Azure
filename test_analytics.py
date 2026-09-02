import unittest
import io
from flight_recorder import FlightRecorder
from analytics.analyzer import Analyzer
from analytics.statistics import Statistics
from analytics.reports import Reports
from analytics.exporters import export_stats_json, export_stats_csv
from analytics.charts import altitude_chart, confidence_chart


class AnalyticsTests(unittest.TestCase):
    def test_statistics_and_report_and_export(self):
        r = FlightRecorder()
        # make 3 steps with estimated state and guidance
        r.record_step(0.0, {"sim_time": 0.0, "altitude": 0.0, "estimated": {"position": (0.0, 0.0, 0.0), "confidence": 0.9}, "mission_status": "EN_ROUTE", "target_altitude": 1.0})
        r.record_step(1.0, {"sim_time": 1.0, "altitude": 0.8, "estimated": {"position": (0.0, 0.0, 1.2), "confidence": 0.8}, "mission_status": "EN_ROUTE", "target_altitude": 1.0, "guidance": {"status": "AVOIDING"}})
        r.record_step(2.0, {"sim_time": 2.0, "altitude": 1.0, "estimated": {"position": (0.0, 0.0, 1.0), "confidence": 0.95}, "mission_status": "MISSION_COMPLETE", "target_altitude": 1.0})

        analyzer = Analyzer(recorder=r)
        entries = analyzer.get_entries()
        stats = Statistics(entries)
        self.assertEqual(stats.mission_completion(), 1.0)
        conf_series = stats.estimator_confidence_over_time()
        self.assertEqual(len(conf_series), 3)
        self.assertEqual(stats.obstacle_avoidance_events(), 1)

        report = Reports.mission_summary(stats)
        self.assertIn("Mission Complete", report)

        # exporters
        sio = io.StringIO()
        export_stats_csv({"overshoot": stats.max_overshoot(), "avg_err": stats.average_altitude_error()}, sio)
        self.assertIn("overshoot", sio.getvalue())
        sio2 = io.StringIO()
        export_stats_json({"a": 1}, sio2)
        self.assertIn('"a": 1', sio2.getvalue())

        # charts produce figures
        alt_series = [(e.sim_time, e.telemetry.get("estimated", {}).get("position", (0,0,e.telemetry.get("altitude")))[2]) for e in entries]
        fig = altitude_chart(alt_series)
        self.assertIsNotNone(fig)
        fig2 = confidence_chart(conf_series)
        self.assertIsNotNone(fig2)

    def test_empty_logs(self):
        r = FlightRecorder()
        analyzer = Analyzer(recorder=r)
        stats = Statistics(analyzer.get_entries())
        self.assertEqual(stats.mission_completion(), 0.0)
        self.assertEqual(stats.obstacle_avoidance_events(), 0)


if __name__ == "__main__":
    unittest.main()
