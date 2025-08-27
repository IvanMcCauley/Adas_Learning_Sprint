#!/usr/bin/env bash
set -e

# Run from repo root: day5_braking/
if [ ! -x build/brake_cli ]; then
  echo "Building brake_cli..."
  mkdir -p build
  ( cd build && cmake .. >/dev/null && make -j >/dev/null )
fi

echo "[1/3] data_ok.csv (default margin)"
./build/brake_cli --csv tests/fixtures/data_ok.csv > tests/out1.csv
diff -u tests/fixtures/expected_ok.csv tests/out1.csv && echo "PASS 1"

echo "[2/3] data_margin.csv (default margin=0.10 -> BRAKE)"
./build/brake_cli --csv tests/fixtures/data_margin.csv > tests/out2.csv
diff -u tests/fixtures/expected_margin_default.csv tests/out2.csv && echo "PASS 2"

echo "[3/3] data_margin.csv (margin=0.0 -> OK)"
./build/brake_cli --csv tests/fixtures/data_margin.csv --margin 0.0 > tests/out3.csv
diff -u tests/fixtures/expected_margin_zero.csv tests/out3.csv && echo "PASS 3"

echo "All CSV tests passed."
