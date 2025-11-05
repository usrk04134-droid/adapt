#!/bin/bash
# Script to rebuild and test after calibration top touch fix

set -e

echo "╔═══════════════════════════════════════════════════════════════════╗"
echo "║          REBUILD AND TEST - Calibration Top Touch Fix            ║"
echo "╚═══════════════════════════════════════════════════════════════════╝"
echo ""

cd "$(dirname "$0")"

echo "🧹 Step 1: Clean old build..."
if [ -d "build" ]; then
    echo "   Removing build/ directory..."
    rm -rf build/
    echo "   ✓ Clean complete"
else
    echo "   No build directory to clean"
fi

echo ""
echo "🔨 Step 2: Rebuild tests..."
echo "   This may take a few minutes..."
./adaptio.sh --build-tests

if [ $? -ne 0 ]; then
    echo "   ✗ Build failed!"
    echo ""
    echo "Please check the build errors above."
    exit 1
fi

echo "   ✓ Build complete"

echo ""
echo "✅ Step 3: Verify image paths..."
cd build/debug

echo "   Running count_test_images..."
./src/adaptio-block-tests --doctest-test-case=count_test_images | grep -E "TEST.*Standard|TEST.*Alternative" || true

echo ""
echo "📊 Step 4: Running calibration tests..."
echo "   (This will test both images with full calibration workflow)"
echo ""

cd ../..
./RUN_IMAGE_TESTS.sh "$@"

echo ""
echo "╔═══════════════════════════════════════════════════════════════════╗"
echo "║                     REBUILD AND TEST COMPLETE                     ║"
echo "╚═══════════════════════════════════════════════════════════════════╝"
