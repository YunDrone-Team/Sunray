#!/usr/bin/env bash

set -euo pipefail

TESTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$TESTS_DIR/.." && pwd)"
OUTPUT_DIR="${SUNRAY_TEST_OUTPUT_DIR:-$TESTS_DIR/output}"

find_dashboard_script() {
  find "$ROOT_DIR" \
    -path "*/sunray_test/scripts/run_test_dashboard.py" \
    -type f \
    -print \
    -quit
}

DASHBOARD_SCRIPT="${SUNRAY_TEST_DASHBOARD_SCRIPT:-$(find_dashboard_script)}"
if [[ -z "$DASHBOARD_SCRIPT" || ! -f "$DASHBOARD_SCRIPT" ]]; then
  echo "Cannot locate run_test_dashboard.py under $ROOT_DIR" >&2
  exit 1
fi

PACKAGE_DIR="$(cd "$(dirname "$DASHBOARD_SCRIPT")/.." && pwd)"
DEPS_SCRIPT="$PACKAGE_DIR/scripts/ensure_runtime_deps.sh"

if [[ "${SUNRAY_TEST_SKIP_DEP_CHECK:-0}" != "1" ]]; then
  bash "$DEPS_SCRIPT"
fi

exec "$DASHBOARD_SCRIPT" \
  --output-dir "$OUTPUT_DIR" \
  "$@"
