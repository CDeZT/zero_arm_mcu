#!/bin/sh
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "=== ZEROARM MCU Comprehensive Test Suite ==="
echo "Project: ${PROJECT_DIR}"
echo ""

BUILD_DIR="${PROJECT_DIR}/build/comprehensive"

echo "[1/3] Configure ..."
cmake -S "${PROJECT_DIR}/Tests" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Debug

echo "[2/3] Build ..."
cmake --build "${BUILD_DIR}" --parallel

echo "[3/3] Run tests ..."
ctest --test-dir "${BUILD_DIR}" --output-on-failure -j4

TOTAL=$(ctest --test-dir "${BUILD_DIR}" -N 2>/dev/null | grep -c 'Test #')
PASSED=$(ctest --test-dir "${BUILD_DIR}" --output-on-failure -j4 2>&1 | grep -c 'Passed')
FAILED=$(ctest --test-dir "${BUILD_DIR}" --output-on-failure -j4 2>&1 | grep -c '\*\*\*')

echo ""
echo "=== Results: ${TOTAL} tests total ==="

echo ""
echo "=== Running ASan/UBSan tests ==="
ASAN_DIR="${PROJECT_DIR}/build/comprehensive-asan"
cmake -S "${PROJECT_DIR}/Tests" -B "${ASAN_DIR}" \
  -DCMAKE_BUILD_TYPE=Debug \
  '-DCMAKE_C_FLAGS=-fsanitize=address,undefined -fno-omit-frame-pointer' \
  '-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined'
cmake --build "${ASAN_DIR}" --parallel
ctest --test-dir "${ASAN_DIR}" --output-on-failure -j4

echo ""
echo "=== All test suites passed ==="
