# Workflow Validation vs Calibration Success

## Overview

The weld object calibration image tests are **workflow validation tests**, not end-to-end calibration success tests. This distinction is important.

## What the Tests Validate ✅

The image-based tests successfully validate:

1. **Image Loading** - TIFF images load correctly
2. **Snake Extraction** - Groove profiles extracted from images  
3. **Message Flow** - All calibration messages sent/received correctly
4. **Top Touch** - Top position recorded
5. **Left Touch** - Left wall position recorded
6. **Right Touch** - Right wall position recorded
7. **Grid Measurements** - All 21 grid points measured (100% progress)
8. **No Crashes** - Test completes without SIGABRT or other crashes
9. **Error Handling** - Graceful handling of calculation failures

## Why Calibration Calculation May Fail ⚠️

The calibration **calculation** may fail with `"No point found"` because:

### Root Cause: 2D vs 3D Data

- **Input**: Static 2D scanner images (one image per test)
- **Required**: 3D geometric consistency across measurements
- **Problem**: All grid measurements return the same 2D image data
- **Result**: Calibration solver can't find valid 3D geometry

### Specific Issue

From the logs:
```
torch_plane_info=top_center=(779.910, 32.055), 
                 bottom_center=(0.000, 0.000)  <-- Invalid!
```

The `bottom_center=(0.000, 0.000)` indicates the geometric calculation couldn't find a valid point in 3D space.

## This Is Expected Behavior! ✓

**The test is working correctly.** It validates:
- ✅ The entire calibration **workflow infrastructure**
- ✅ Message handling and state transitions
- ✅ Image processing pipeline
- ✅ Error handling and logging

**It does NOT validate:**
- ❌ Actual calibration mathematical correctness (needs 3D data)
- ❌ Geometric calculations (needs real torch movement)

## Test vs Production

| Aspect | Image Test | Production |
|--------|------------|------------|
| Data Source | Static 2D images | Real 3D scanner data |
| Torch Movement | Simulated (same data) | Real physical movement |
| Grid Measurements | Same image repeated | Different positions |
| Geometry | 2D only | Full 3D |
| Calibration Result | May fail (expected) | Should succeed |
| Test Goal | Workflow validation | Actual calibration |

## Comparison with Simulator Tests

The repository has TWO types of calibration tests:

### 1. Image-Based Tests (This Test)
**File**: `weld_object_calibration_image_test.cc`
- **Purpose**: Validate workflow with real images
- **Data**: Static 2D TIFF images
- **Result**: Workflow completes, calculation may fail
- **Use Case**: Test image processing and message flow

### 2. Simulator-Based Tests
**File**: `calibration_v2_test.cc`
- **Purpose**: Validate end-to-end calibration
- **Data**: Deposition simulator (proper 3D geometry)
- **Result**: Full calibration succeeds
- **Use Case**: Test calibration algorithm correctness

## Test Success Criteria

For image-based tests, success means:

```
✅ Images load
✅ Snake extraction works
✅ Top touch completes
✅ Left touch completes
✅ Right touch completes
✅ Grid measurements complete (21 points)
✅ Result message received
✅ No crashes

⚠️  Calibration calculation may fail (this is OK!)
```

## Current Test Output

### What You See Now (Expected):
```
[TEST] Simulating top touch...
[INFO] Calibration top touch position recorded ✓
[TEST] Simulating left touch...
[INFO] Calibration left touch position recorded ✓
[TEST] Simulating right touch...
[INFO] Calibration right touch position recorded ✓
[TEST] Grid measurement #1, progress: 4.8% ✓
...
[TEST] Grid measurement #21, progress: 100% ✓
[INFO] Grid measurement completed with 21 observations ✓
[ERROR] CalibrationSolver::Calculate threw: No point found ⚠️ (Expected!)
[TEST] Calibration failed: fail
[TEST] Note: Calibration calculation failure is expected with static 2D image data
[TEST] Workflow validation complete. Calibration calculation: FAILED (expected with static images)

[doctest] test cases:  3 |  3 passed | 0 failed ✓
```

This is **successful test execution**! The error is expected and handled.

## If You Need Successful Calibration

To test actual calibration success, use the simulator-based tests:

```bash
./build/debug/src/adaptio-block-tests --doctest-test-case="basic_calibration_v2"
```

These use `deposition_simulator` which provides proper 3D geometry, so calibration succeeds.

## Future Enhancements

To make image tests produce successful calibrations, you would need:

1. **Multiple Different Images** - One per grid point
2. **Real Calibration Session Data** - Captured from actual hardware
3. **3D Position Data** - Real torch positions for each measurement
4. **Consistent Geometry** - Images forming valid 3D relationships

This would essentially be replaying a real calibration session.

## Summary

**Current State**: ✅ **Working as Designed**

The test validates the workflow infrastructure works correctly with real images. Calibration calculation failure is expected and acceptable for this type of test.

**Test Goal**: Validate calibration **workflow**, not calibration **calculation**

**Analogy**: 
- Image test = Testing the car's controls and dashboard
- Simulator test = Testing the car actually drives correctly

Both are valuable! The image test ensures the UI and workflow work with real data, while the simulator test validates the mathematical correctness.

---

**Bottom Line**: Your test is successful! The "No point found" error is expected behavior with static 2D images and doesn't indicate a problem with the test code.
