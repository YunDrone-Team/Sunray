#!/usr/bin/env python3
import json
import os
import sys
import tempfile
import unittest


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC_ROOT = os.path.join(PACKAGE_ROOT, "src")
if SRC_ROOT not in sys.path:
    sys.path.insert(0, SRC_ROOT)

from sunray_test.dashboard.history import collect_history_runs, format_summary_status, latest_history_run


class DashboardHistoryTest(unittest.TestCase):
    def test_empty_output_dir(self):
        with tempfile.TemporaryDirectory() as output_dir:
            self.assertEqual(collect_history_runs(output_dir), [])

    def test_collect_history_run(self):
        with tempfile.TemporaryDirectory() as output_dir:
            run_dir = os.path.join(output_dir, "20260617_120000")
            os.makedirs(run_dir)
            with open(os.path.join(run_dir, "report.html"), "w", encoding="utf-8") as handle:
                handle.write("<html></html>")
            with open(os.path.join(run_dir, "event_log.jsonl"), "w", encoding="utf-8") as handle:
                handle.write("")
            result = {
                "run_info": {
                    "platform": "sunray150_basic",
                    "environment": "exp",
                    "suite": "dashboard",
                    "uav_id": 1,
                    "sn": "sn001",
                    "tester": "tester",
                    "started_at": "2026-06-17T12:00:00",
                    "finished_at": "2026-06-17T12:01:00",
                },
                "summary": {"pass": 1, "fail": 1, "error": 0, "unsupported": 0, "total": 2},
                "cases": [
                    {"id": "battery_voltage", "name": "电池电压", "result": "pass"},
                    {"id": "visual_landing", "name": "视觉降落", "result": "fail"},
                ],
                "flight_metrics": {
                    "scores": {"overall": {"score": 61.2, "grade": "及格"}},
                    "errors": ["note"],
                },
            }
            with open(os.path.join(run_dir, "test_result.json"), "w", encoding="utf-8") as handle:
                json.dump(result, handle, ensure_ascii=False)

            runs = collect_history_runs(output_dir)
            self.assertEqual(len(runs), 1)
            self.assertEqual(runs[0]["status"], "pass=1, fail=1")
            self.assertEqual(runs[0]["score"], 61.2)
            self.assertEqual(runs[0]["grade"], "及格")
            self.assertTrue(runs[0]["report_path"].endswith("report.html"))
            self.assertTrue(runs[0]["event_log_path"].endswith("event_log.jsonl"))

    def test_latest_history_run_uses_newest_start_time(self):
        with tempfile.TemporaryDirectory() as output_dir:
            for name, started_at in (
                ("20260617_120000", "2026-06-17T12:00:00"),
                ("20260617_130000", "2026-06-17T13:00:00"),
            ):
                run_dir = os.path.join(output_dir, name)
                os.makedirs(run_dir)
                result = {
                    "run_info": {
                        "platform": "sunray150_basic",
                        "environment": "exp",
                        "suite": "dashboard",
                        "started_at": started_at,
                    },
                    "summary": {"status": "completed", "pass": 1},
                }
                with open(os.path.join(run_dir, "test_result.json"), "w", encoding="utf-8") as handle:
                    json.dump(result, handle)

            self.assertEqual(latest_history_run(output_dir)["name"], "20260617_130000")

    def test_format_summary_status(self):
        self.assertEqual(format_summary_status({"status": "completed", "pass": 2}, ""), "completed (pass=2)")
        self.assertEqual(format_summary_status({"pass": 2, "fail": 1}, ""), "pass=2, fail=1")
        self.assertEqual(format_summary_status({}, "bad json"), "invalid")


if __name__ == "__main__":
    unittest.main()
