#!/usr/bin/env python3
import os
import sys
import unittest


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC_ROOT = os.path.join(PACKAGE_ROOT, "src")
if SRC_ROOT not in sys.path:
    sys.path.insert(0, SRC_ROOT)

from sunray_test.reports.renderers.flight import render_config_snapshot
from sunray_test.reports.renderers.summary import render_summary_cards


class ReportRenderersTest(unittest.TestCase):
    def test_config_snapshot_renders_expandable_sections_with_function_groups(self):
        html = render_config_snapshot(
            {
                "defaults": {"takeoff_target_z_m": 1.2},
                "analysis": {"pose_topic": "/uav1/mavros/local_position/pose"},
                "topics": {"battery": "/uav1/mavros/battery"},
                "missions": {"waypoint": {"name": "航点", "waypoints": [[0.5, -1.0, 1.2], [0.0, 0.0, 1.2]]}},
            }
        )

        self.assertIn('class="config-snapshot-layout"', html)
        self.assertIn('class="config-snapshot-group"', html)
        self.assertEqual(html.count('<details class="config-snapshot-section">'), 3)
        self.assertNotIn('<details class="config-snapshot-section" open>', html)
        self.assertIn("基础配置", html)
        self.assertIn("话题映射", html)
        self.assertIn("任务详情", html)
        self.assertIn("起飞流程", html)
        self.assertIn("分析输入", html)
        self.assertIn("话题映射", html)
        self.assertIn('class="mission-point-grid"', html)
        self.assertIn('class="mission-point-item"', html)
        self.assertIn('class="mission-point-axis">X</span>', html)
        self.assertIn('class="mission-point-axis">Y</span>', html)
        self.assertIn('class="mission-point-axis">Z</span>', html)
        self.assertIn('class="mission-point-number">0.5</span>', html)
        self.assertNotIn("[0.5,-1.0,1.2]", html)

    def test_summary_hides_unsupported_card_when_zero(self):
        html = render_summary_cards(
            {"total": 2, "pass": 2, "fail": 0, "error": 0, "unsupported": 0},
            100.0,
        )

        self.assertNotIn("不支持", html)
        self.assertNotIn("summary-unsupported", html)
        self.assertIn("通过率", html)

    def test_summary_shows_unsupported_card_when_present(self):
        html = render_summary_cards(
            {"total": 3, "pass": 2, "fail": 0, "error": 0, "unsupported": 1},
            66.7,
        )

        self.assertIn("不支持", html)
        self.assertIn("summary-unsupported", html)


if __name__ == "__main__":
    unittest.main()
