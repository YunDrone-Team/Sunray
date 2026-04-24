#!/bin/bash

set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
"$ROOT_DIR/../General_Module/sunray_test/scripts/run_scenario.py" --scenario sunray150_basic_sim
