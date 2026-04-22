#!/usr/bin/env python3
import argparse
import json
import os
import sys


SCRIPT_DIR = os.path.abspath(os.path.dirname(__file__))
PACKAGE_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
SRC_ROOT = os.path.join(PACKAGE_ROOT, "src")
if SRC_ROOT not in sys.path:
    sys.path.insert(0, SRC_ROOT)

from sunray_test.reports.html_renderer import render_html
from sunray_test.reports.flight_metrics import enrich_report_payload


def parse_args():
    parser = argparse.ArgumentParser(description="Generate HTML report from test_result.json")
    parser.add_argument("--json-path", required=True)
    parser.add_argument("--output-path", default="")
    return parser.parse_args()


def main():
    args = parse_args()
    with open(args.json_path, "r", encoding="utf-8") as handle:
        payload = json.load(handle)
    payload = enrich_report_payload(payload)
    with open(args.json_path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2)
    output_path = args.output_path or os.path.join(os.path.dirname(args.json_path), "report.html")
    with open(output_path, "w", encoding="utf-8") as handle:
        handle.write(render_html(payload))
    print(f"report generated: {output_path}")


if __name__ == "__main__":
    main()
