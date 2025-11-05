# Calibration Top Touch Fix

## Problem

The weld object calibration was failing with error:
```
[error] - calibration_manager_impl.cc:574 | No top observation available
[error] - calibration_manager_impl.cc:559 | Calibration right touch procedure failed: top_center calculation failed
```

## Root Cause

The calibration workflow requires **THREE touch points** in this order:
1. **Top touch** - Touch the top center of the groove
2. **Left touch** - Touch the left wall
3. **Right touch** - Touch the right wall

Our test was only providing left and right touches, missing the required top touch.

## Solution Applied

Added the top touch step to the calibration workflow:

```cpp
// STEP 1: Top touch - required for calibration
double top_touch_horizontal = (slice_data.groove[0].x + slice_data.groove[6].x) / 2.0;
double top_touch_vertical = std::min(slice_data.groove[0].y, slice_data.groove[6].y) - 5.0;

WeldObjectCalTopPos(fixture);
ProvideImageBasedScannerData(fixture, slice_data, top_touch_horizontal, top_touch_vertical);
CHECK(WeldObjectCalTopPosRsp(fixture));

// STEP 2: Left touch
// ... (existing code)

// STEP 3: Right touch
// ... (existing code)
```

## Required: Rebuild Tests

**IMPORTANT**: You MUST rebuild the tests for the fix to take effect!

```bash
cd /workspace/adaptio

# Clean build (recommended)
rm -rf build/

# Rebuild tests
./adaptio.sh --build-tests
```

## Run Tests After Rebuild

```bash
cd /workspace/adaptio

# Option 1: Use helper script
./RUN_IMAGE_TESTS.sh

# Option 2: Manual from build directory
cd build/debug
./src/adaptio-block-tests --doctest-test-suite=WeldObjectCalibrationImage
```

## Calibration Flow (Complete)

The correct calibration sequence is now:

```
1. Start Calibration
   └─ WeldObjectCalStart
   
2. Top Touch (NEW!)
   ├─ Operator positions wire at top center
   ├─ WeldObjectCalTopPos
   └─ System records top position
   
3. Left Touch
   ├─ Operator positions wire touching left wall
   ├─ WeldObjectCalLeftPos
   └─ System records left position
   
4. Right Touch
   ├─ Operator positions wire touching right wall
   ├─ WeldObjectCalRightPos
   └─ System calculates calibration
   
5. Automatic Grid Measurement
   └─ System moves to grid points and collects data
   
6. Calibration Complete
   └─ WeldObjectCalResult returned
```

## Expected Success Output

After rebuild and running tests, you should see:

```
[TEST] Simulating top touch at horizontal: XXX.XX, vertical: YY.YY
[INFO] WeldObjectCalTopPos received
[INFO] Calibration top touch position, recorded at h: XXX.XX, v: YY.YY

[TEST] Simulating left touch at horizontal: XXX.XX, vertical: YY.YY
[INFO] WeldObjectCalLeftPos received
[INFO] Calibration left touch position, recorded at h: XXX.XX, v: YY.YY

[TEST] Simulating right touch at horizontal: XXX.XX, vertical: YY.YY
[INFO] WeldObjectCalRightPos received
[INFO] Calibration right touch position, recorded at h: XXX.XX, v: YY.YY

[TEST] Automatic grid measurement sequence started
[TEST] Grid measurement #1, progress: X.X%
...
[TEST] Calibration successful!

[doctest] test cases:  3 |  3 passed | 0 failed
[doctest] assertions: 21 | 21 passed | 0 failed
```

## Additional Note: Image Paths

I noticed in your test output the image paths shown are:
```
./src/scanner/joint_model/test/test_data/1761811185215.tiff
```

But the code has:
```
../tests/configs/sil/calibration/1738232679592.tiff
```

This confirms you're running an **OLD COMPILED VERSION**. The rebuild is essential!

## Verification Steps

After rebuilding:

1. **Verify paths are correct:**
   ```bash
   cd build/debug
   ./src/adaptio-block-tests --doctest-test-case=count_test_images
   ```
   
   Should show:
   ```
   [0] Standard calibration image - 3500x500 -> ../tests/configs/sil/calibration/1738232679592.tiff
   [1] Alternative calibration image - 3500x520 -> ../tests/configs/sil/1738243625597.tiff
   ```

2. **Check images load:**
   ```bash
   cd build/debug
   ./src/adaptio-block-tests --doctest-test-case=image_loading_test_all
   ```
   
   Both images should load successfully with snake extraction.

3. **Run full calibration:**
   ```bash
   ./RUN_IMAGE_TESTS.sh --trace
   ```
   
   Should complete all 3 test cases successfully.

## Files Modified

- `src/block_tests/weld_object_calibration_image_test.cc` - Added top touch step

## Summary

**Before fix:**
- Only 2 touches (left, right)
- Calibration failed with "No top observation available"

**After fix:**
- 3 touches (top, left, right)
- Calibration should complete successfully

**Action Required:**
```bash
cd /workspace/adaptio
rm -rf build/
./adaptio.sh --build-tests
./RUN_IMAGE_TESTS.sh
```
