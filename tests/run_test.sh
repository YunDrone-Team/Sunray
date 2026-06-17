#!/bin/bash

set -e

TESTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$TESTS_DIR/.." && pwd)"
OUTPUT_DIR="$TESTS_DIR/output"

exec "$ROOT_DIR/General_Module/sunray_test/scripts/run_test_dashboard.py" \
  --output-dir "$OUTPUT_DIR" \
  "$@"
