#!/usr/bin/env bash
# Stage-1 setup: pure-Python reference simulation (what CI actually runs).
set -euo pipefail
cd "$(dirname "$0")/.."
python3 -m pip install -r requirements.txt
python3 -m pytest tests -q
echo "--- component conformance checks ---"
gcc -std=c11 -Wall -Wextra -pedantic -o /tmp/abi_check components/c/sensor_abi_conformance.c -lm && /tmp/abi_check
g++ -std=c++17 -Wall -Wextra -pedantic -o /tmp/fc components/cpp/flight_controller.cpp && /tmp/fc
node --check components/javascript/telemetry_printer.js
python3 - <<'PY'
import sqlite3
conn = sqlite3.connect(":memory:")
conn.executescript(open("components/sql/telemetry_schema.sql").read())
print("SQL schema OK")
PY
echo "ALL CHECKS PASSED"
