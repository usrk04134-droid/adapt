#!/bin/bash
# Helper script to run weld object calibration image tests
# This ensures tests are run from the correct directory

set -e

echo "=========================================="
echo "Weld Object Calibration Image Tests"
echo "=========================================="

# Change to repository root
cd "$(dirname "$0")"
REPO_ROOT=$(pwd)

echo "Repository root: $REPO_ROOT"

# Check if build directory exists
if [ ! -d "build/debug" ]; then
    echo "ERROR: build/debug directory not found"
    echo "Please build the tests first:"
    echo "  ./adaptio.sh --build-tests"
    exit 1
fi

# Check if test binary exists
if [ ! -f "build/debug/src/adaptio-block-tests" ]; then
    echo "ERROR: Test binary not found at build/debug/src/adaptio-block-tests"
    echo "Please build the tests first:"
    echo "  ./adaptio.sh --build-tests"
    exit 1
fi

# Verify test images exist
echo ""
echo "Checking for test images..."
cd build/debug

if [ -f "../tests/configs/sil/calibration/1738232679592.tiff" ]; then
    echo "✓ Found: ../tests/configs/sil/calibration/1738232679592.tiff"
else
    echo "✗ Missing: ../tests/configs/sil/calibration/1738232679592.tiff"
fi

if [ -f "../tests/configs/sil/1738243625597.tiff" ]; then
    echo "✓ Found: ../tests/configs/sil/1738243625597.tiff"
else
    echo "✗ Missing: ../tests/configs/sil/1738243625597.tiff"
fi

echo ""
echo "Running tests from: $(pwd)"
echo "=========================================="
echo ""

# Run the tests
./src/adaptio-block-tests "$@" --doctest-test-suite=WeldObjectCalibrationImage

echo ""
echo "=========================================="
echo "Test run complete"
echo "=========================================="
