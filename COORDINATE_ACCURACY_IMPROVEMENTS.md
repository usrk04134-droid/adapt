# Block Test Coordinate Accuracy Improvements

## Summary

Improved coordinate accuracy checks in calibration and joint_tracking block tests from 10mm tolerance to 1mm tolerance, and fixed systematic coordinate errors.

## Changes Made

### 1. Tolerance Improvements (All Test Cases)

Updated coordinate tolerance from **10mm (0.01m) to 1mm (0.001m)** in:

**Files Modified:**
- `src/block_tests/calibration_test.cc`
  - `basic_calibration` test (lines 96-98)
  - `basic_calibration_touch_top_u_bevel` test (lines 131-133)
  - `lw_calibration` test (lines 166-168)
  
- `src/block_tests/joint_tracking_test.cc`
  - `basic_sequence` test (lines 72-74)

**Change:**
```cpp
// Before
const double tolerance_m = 0.01;  // 10mm tolerance

// After  
const double tolerance_m = 0.001;  // 1mm tolerance
```

### 2. Bidirectional Coordinate Checks (All Test Cases)

Added `std::abs()` to properly check coordinate differences in both positive and negative directions for both horizontal (X-axis) and vertical (Z-axis) coordinates.

**Change:**
```cpp
// Before - Only checks if positive difference is within tolerance
CHECK((final_torch_pos.GetX() - expected_x) < tolerance_m);
CHECK((final_torch_pos.GetZ() - expected_z) < tolerance_m);

// After - Checks absolute difference (both positive and negative)
CHECK(std::abs(final_torch_pos.GetX() - expected_x) < tolerance_m);
CHECK(std::abs(final_torch_pos.GetZ() - expected_z) < tolerance_m);
```

### 3. Fixed Vertical Offset Systematic Error

Removed extra 1mm offset that was causing systematic coordinate inaccuracy in vertical positioning.

**calibration_test.cc (line 47):**
```cpp
// Before
const float JT_VERTICAL_OFFSET = STICKOUT_M * 1000 + 1.0;  // 26mm (25mm + 1mm extra)

// After  
const float JT_VERTICAL_OFFSET = STICKOUT_M * 1000;  // 25mm (correct)
```

**joint_tracking_test.cc (line 65):**
```cpp
// Before
const float jt_vertical_offset = static_cast<float>(STICKOUT_M * 1000 + 1.0);  // 26mm

// After
const float jt_vertical_offset = static_cast<float>(STICKOUT_M * 1000);  // 25mm
```

### 4. Created Missing Helper File

Created `src/block_tests/helpers/helpers_mfx_tracking.h` which was referenced but missing. This file contains the `JointTracking()` helper function that:
- Starts joint tracking via controller
- Provides scanner and kinematics data
- Iterates until position converges
- Updates simulator torch position based on controller output

## Impact

These changes ensure:

1. **10x improvement in accuracy requirements**: Tests now require ~1mm accuracy instead of ~10mm
2. **Proper bidirectional validation**: Both positive and negative coordinate deviations are caught
3. **Eliminated systematic error**: Removed 1mm vertical offset bug
4. **Better test reliability**: More stringent tests catch coordinate inaccuracies earlier in development

## Testing

To rebuild and run the affected tests:

```bash
# Build tests
./adaptio.sh --build-tests

# Run all block tests
./build/debug/src/adaptio-block-tests --trace

# Run specific test suites
./build/debug/src/adaptio-block-tests --trace --doctest-test-suite="MultiblockCalibration"
./build/debug/src/adaptio-block-tests --trace --doctest-test-suite="Joint_tracking"

# Run specific test cases
./build/debug/src/adaptio-block-tests --trace --doctest-test-case="basic_calibration"
./build/debug/src/adaptio-block-tests --trace --doctest-test-case="lw_calibration"
./build/debug/src/adaptio-block-tests --trace --doctest-test-case="basic_sequence"
```

## Files Modified

1. `src/block_tests/calibration_test.cc` - Updated 3 test cases with improved accuracy
2. `src/block_tests/joint_tracking_test.cc` - Updated 1 test case with improved accuracy  
3. `src/block_tests/helpers/helpers_mfx_tracking.h` - Created new helper file

## Expected Results

After these changes, the block tests will:
- Verify horizontal (X) coordinate accuracy within ±1mm
- Verify vertical (Z) coordinate accuracy within ±1mm
- Catch any systematic coordinate errors > 1mm
- Ensure expected and final torch positions match within 1mm tolerance
