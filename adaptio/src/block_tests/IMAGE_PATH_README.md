# Image Path Configuration for Block Tests

## Issue: Image Not Found Errors

If you see errors like:
```
Failed to load image: ./tests/configs/sil/calibration/1738232679592.tiff
can't open/read file: check file path/integrity
```

## Solution: Correct Working Directory

The block tests expect to be run from the **build directory**, not the repository root.

### Correct Usage

```bash
# From repository root
cd /workspace/adaptio

# Build tests
./adaptio.sh --build-tests

# Run from BUILD directory (important!)
cd build/debug
./src/adaptio-block-tests --doctest-test-suite=WeldObjectCalibrationImage

# Or use relative path from build dir
cd build/debug && ../../scripts/run-tests.sh
```

### Why This Matters

The test image paths in `weld_object_calibration_image_test.cc` are configured as:
```cpp
"../tests/configs/sil/calibration/1738232679592.tiff"
```

This path is **relative to the build directory** (`build/debug/`):
```
build/debug/                         <- Run tests from here
    └── ../tests/configs/...         <- This resolves correctly
        = tests/configs/...
```

## Verifying Image Paths

Before running tests, verify images exist:
```bash
# From build/debug directory
ls -la ../tests/configs/sil/calibration/*.tiff
ls -la ../tests/configs/sil/*.tiff
```

Expected output:
```
../tests/configs/sil/calibration/1738232679592.tiff
../tests/configs/sil/1738243625597.tiff
```

## Adding New Test Images

When adding new images to `TEST_IMAGES` vector, use paths relative to **build directory**:

```cpp
const std::vector<TestImageConfig> TEST_IMAGES = {
    // Correct - relative to build/debug/
    TestImageConfig(
        "../tests/configs/sil/my_image.tiff",
        "My test image"
    ),
    
    // WRONG - would only work if running from repo root
    // TestImageConfig("./tests/configs/sil/my_image.tiff", ...),
};
```

## Common Errors and Fixes

### Error: "can't open/read file"
**Cause**: Running tests from wrong directory  
**Fix**: `cd build/debug` before running tests

### Error: Image path shows unexpected location
**Cause**: TEST_IMAGES vector has wrong paths  
**Fix**: Update paths to use `../tests/...` format

### Error: Tests work locally but fail in CI
**Cause**: CI might run from different directory  
**Fix**: Ensure CI scripts cd into build directory before running tests

## Quick Test

To verify your setup:
```bash
# Run the count_test_images test case
cd build/debug
./src/adaptio-block-tests --doctest-test-case=count_test_images

# This will list configured images and their paths
# Check that paths are correct
```

## Alternative: Absolute Paths

If relative paths cause issues, you can use absolute paths in your local testing:

```cpp
// For local testing only - don't commit
TestImageConfig(
    "/workspace/adaptio/tests/configs/sil/calibration/1738232679592.tiff",
    "Test image"
),
```

**Note**: Absolute paths should only be used for local debugging, not committed to the repository.
