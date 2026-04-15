#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR_NAME="${1:-build}"
TEST_DIR="$ROOT_DIR/$BUILD_DIR_NAME/tests"

if [[ ! -d "$TEST_DIR" ]]; then
  echo "Test directory not found: $TEST_DIR"
  echo "Build first: cmake -S . -B $BUILD_DIR_NAME && cmake --build $BUILD_DIR_NAME"
  exit 2
fi

fail=0
for t in "$TEST_DIR"/*GTEST; do
  name="$(basename "$t")"
  log="/tmp/vg_${name}.log"
  echo "=== $name ==="

  if valgrind \
    --leak-check=full \
    --show-leak-kinds=definite,indirect,possible \
    --errors-for-leak-kinds=definite,indirect,possible \
    --error-exitcode=42 \
    "$t" >"$log" 2>&1; then
    echo "OK"
  else
    rc=$?
    echo "FAIL rc=$rc"
    fail=1
  fi

  awk '/ERROR SUMMARY|definitely lost|indirectly lost|possibly lost/' "$log" || true
done

exit "$fail"
