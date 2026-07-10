#!/bin/bash
# run_all_tests.sh — Run all tests for the polctrl project.
#
# Includes:
#   1. Build C library (libpolctrl.so)
#   2. Run pytest test suite
#   3. Grep checks: no float/double in algorithm core
#   4. Grep checks: no malloc/free in algorithm core
#
# Exit code 0 = all pass, non-zero = failure.

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
C_DIR="$PROJECT_DIR/c"

echo "========================================"
echo "  PolCtrl Test Suite"
echo "========================================"

# --- 1. Build C library ---
echo ""
echo "[1/4] Building libpolctrl.so..."
cd "$C_DIR"
make clean > /dev/null 2>&1
make 2>&1
if [ $? -ne 0 ]; then
    echo "FAIL: C build failed"
    exit 1
fi
echo "OK: C library built"

# --- 2. Run pytest ---
echo ""
echo "[2/4] Running pytest..."
cd "$PROJECT_DIR"
python3 -m pytest tests/ -v 2>&1
if [ $? -ne 0 ]; then
    echo "FAIL: pytest tests failed"
    exit 1
fi
echo "OK: All pytest tests passed"

# --- 3. Grep: no float/double in algorithm core ---
echo ""
echo "[3/4] Checking for float/double in algorithm core..."
CORE_FILES="$C_DIR/src/baseline.c $C_DIR/src/spsa.c $C_DIR/src/fsm.c $C_DIR/src/bandit.c $C_DIR/src/polctrl.c"
FLOAT_FOUND=0
for f in $CORE_FILES; do
    # Search for float/double keywords (not in comments)
    MATCHES=$(grep -nE '\b(float|double)\b' "$f" | grep -v '^\s*//' | grep -v '^\s*\*' | grep -v '/\*' || true)
    if [ -n "$MATCHES" ]; then
        echo "FAIL: float/double found in $f:"
        echo "$MATCHES"
        FLOAT_FOUND=1
    fi
done
if [ $FLOAT_FOUND -ne 0 ]; then
    echo "FAIL: float/double detected in algorithm core (not allowed on FPU-less MCU)"
    exit 1
fi
echo "OK: No float/double in algorithm core"

# --- 4. Grep: no malloc/free in algorithm core ---
echo ""
echo "[4/4] Checking for malloc/free in algorithm core..."
MALLOC_FOUND=0
for f in $CORE_FILES $C_DIR/src/fixedpoint.c $C_DIR/src/rng.c; do
    MATCHES=$(grep -nE '\b(malloc|calloc|realloc|free)\b' "$f" | grep -v '^\s*//' | grep -v '^\s*\*' | grep -v '/\*' || true)
    if [ -n "$MATCHES" ]; then
        echo "FAIL: malloc/free found in $f:"
        echo "$MATCHES"
        MALLOC_FOUND=1
    fi
done
if [ $MALLOC_FOUND -ne 0 ]; then
    echo "FAIL: dynamic allocation detected in algorithm core (not allowed on MCU)"
    exit 1
fi
echo "OK: No malloc/free in algorithm core"

echo ""
echo "========================================"
echo "  ALL TESTS PASSED"
echo "========================================"
