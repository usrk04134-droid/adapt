# LW Calibration Handler Analysis

## Overview

The `LWCalibrationHandler` is a new implementation for **Longitudinal Welding (LW)** calibration, which differs significantly from the existing `CalibrationManagerImpl` used for circular welding calibration.

## Key Differences from Existing Calibration

### 1. **Simplified Calibration Flow**

**Existing (Circular Welding):**
```
Start → Top → Left → Right → Generate Grid → Automated Sequence (multiple points) → Compute Result
```

**New (Longitudinal Welding):**
```
Start → Initial Observation → Top → Left → Right → Compute Result
```

**Impact:** LW calibration is much simpler - no grid generation, no automated sequence runner. Uses only 4 observations total (1 initial + 3 touch points).

### 2. **Initial Observation Capture**

**New Feature:** After scanner starts successfully, the LW handler automatically captures an "initial observation" with HIGH confidence requirement:

```cpp
calibration_initial_observation_procedure_ = [this](const Observation& observation) {
  cal_ctx_.single_observation = observation;
  // ...
};
```

**Existing:** No automatic initial observation - all observations are manual touch points.

**Impact:** This adds complexity to the start procedure and requires HIGH confidence scanner data immediately after start.

### 3. **Missing Calibration Solver**

**Existing:**
- Uses `CalibrationSolver::Calculate()` with:
  - TorchPlaneInfo
  - GeometricConstants
  - Multiple observations (grid points)

**New:**
- `TryComputeCalibrationResult()` is a **dummy implementation**:
  ```cpp
  auto LWCalibrationHandler::TryComputeCalibrationResult() -> std::optional<LWCalibrationResult> {
    // Dummy calculation for now
    LWCalibrationResult result;
    result.torch_to_lpcs_translation = common::Vector3D{0.0, 0.0, 0.0};
    return result;
  }
  ```

**Critical Issue:** The calibration computation is not implemented! It always returns a zero translation vector, which is useless for actual calibration.

### 4. **Different Storage Type**

**Existing:**
- Uses `StoredCalibrationResult`
- Stores: rotation axis, rotation center, torch-to-LPCS translation, RSE, weld object radius

**New:**
- Uses `StoredLWCalResult` (new type, may not exist)
- Stores different parameters (exact structure unknown)

**Impact:** Requires new storage class implementation and database schema.

### 5. **No Grid Generation or Sequence Runner**

**Existing:**
- Generates grid of calibration points
- Uses `CalibrationSequenceRunner` for automated movement
- Collects many observations for better accuracy

**New:**
- Only uses 4 observations total
- No automated movement
- Simpler but potentially less accurate

### 6. **Different Input Parameters**

**Existing:**
```cpp
payload.at("wireDiameter").get_to(params.wire_diameter);
payload.at("stickout").get_to(params.stickout);
payload.at("weldObjectRadius").get_to(params.weld_object_radius);
```

**New:**
```cpp
payload.at("distanceLaserTorch").get_to(cal_ctx_.distance_laser_torch);
payload.at("stickout").get_to(cal_ctx_.stickout);
payload.at("scannerMountAngle").get_to(cal_ctx_.scanner_mount_angle);
```

**Impact:** Different calibration parameters suggest different calibration methodology.

### 7. **Confidence Level Requirements**

**Existing:**
- Left/Right positions: Requires `confidence != NO` (accepts LOW, MEDIUM, HIGH)
- Grid points: Uses whatever confidence is available

**New:**
- Initial observation: Requires `confidence == HIGH` (strict requirement)
- Left/Right positions: Requires `confidence != NO` (same as existing)

**Impact:** Initial observation may fail if scanner doesn't immediately provide HIGH confidence data.

## Drawbacks of LW Calibration Implementation

### 1. **CRITICAL: Dummy Calibration Computation**

The `TryComputeCalibrationResult()` method is not implemented - it always returns zeros:

```cpp
result.torch_to_lpcs_translation = common::Vector3D{0.0, 0.0, 0.0};
```

**Impact:**
- Calibration will always "succeed" but produce invalid results
- No actual calibration is performed
- System will think it's calibrated but won't work correctly

**Fix Required:** Implement actual calibration algorithm or integrate with existing solver.

### 2. **Missing Validation**

There's a TODO comment indicating missing validation:
```cpp
// TODO: Validate that joint_geometry.type == "lw"
```

**Impact:**
- Wrong joint geometry type could be used
- Calibration might fail silently or produce incorrect results
- No type safety

### 3. **Resource Leak in Error Path**

In `HandleRightPosData()`, after computing the result, it calls:
```cpp
dep_ctx_->kinematics_client->Release();
```

But this is only called in the success path. If `TryComputeCalibrationResult()` fails or `ReportCalibrationResult()` throws, the kinematics client is never released.

**Impact:** Resource leak if computation fails.

### 4. **Inconsistent Cleanup (Same as CW Handler)**

The `Cleanup()` method doesn't fully clean up:
- Doesn't stop scanner
- Doesn't reset activity status
- Doesn't release kinematics client

Cleanup logic is duplicated in:
- `HandleInitialObservationData` (timeout)
- `HandleTopPosData` (timeout)
- `HandleLWLeftTouchFailure`
- `HandleLWRightTouchFailure`
- `HandleRightPosData` (success path)
- `OnLWCalStop`

**Impact:** Code duplication, maintenance burden, potential for inconsistencies.

### 5. **Initial Observation Timing Issue**

The initial observation is captured immediately after scanner starts, but:
- Scanner may not be stable yet
- Groove detection may not be ready
- Requires HIGH confidence immediately

**Impact:** High failure rate for initial observation, leading to calibration start failures.

### 6. **No Error Recovery**

If initial observation fails:
- Calibration start fails
- User must restart entire process
- No way to retry just the initial observation

**Impact:** Poor user experience, requires full restart on transient failures.

### 7. **Missing Top Center Calculation**

Unlike the existing calibration which calculates top center from touch points:
```cpp
const auto top_center = CalculateTopCenter();
```

The LW calibration doesn't calculate or use a top center point. It's unclear how the calibration result is computed without this reference point.

**Impact:** May produce incorrect calibration results.

### 8. **No Quality Metrics**

**Existing:**
- Computes RSE (Residual Standard Error)
- Validates RSE < 1.0 before accepting result
- Provides quality metrics (R², residuals, etc.)

**New:**
- No quality metrics computed
- No validation of result quality
- Always accepts result (even if it's zeros)

**Impact:** No way to detect bad calibrations, system may use invalid calibration data.

### 9. **Missing Model Configuration Update**

**Existing:**
- After successful calibration, updates `ModelConfig`:
  ```cpp
  model_config_->Set(cal_data.value().RotationCenter(), scanner_angles, ...);
  ```

**New:**
- Only stores result, doesn't update model configuration
- `calibration_status_subscriber_()` is called, but model may not be configured

**Impact:** Calibration result stored but not applied to the system.

### 10. **Incomplete Logging**

**Existing:**
- Logs comprehensive data:
  - TorchPlaneInfo
  - GeometricConstants
  - Grid configuration
  - Runner configuration
  - All observations
  - Calibration result with metrics

**New:**
- Logs minimal data:
  - Joint geometry
  - 3 observations (single, left, right)
  - Calibration result (if any)
- Missing: geometric constants, configuration parameters, quality metrics

**Impact:** Less useful for debugging and analysis.

### 11. **No Progress Reporting**

**Existing:**
- Reports progress during automated sequence:
  ```cpp
  web_hmi_->Send("WeldObjectCalProgress", ..., {"progress", progress});
  ```

**New:**
- No progress reporting
- User has no feedback during calibration

**Impact:** Poor user experience, no visibility into calibration progress.

### 12. **Potential Race Condition**

The initial observation procedure is set up in the scanner start callback, but scanner data might arrive before the procedure is set:

```cpp
calibration_start_procedure_ = [this](bool success) {
  if (success) {
    SetProcedureStartTime();
    calibration_initial_observation_procedure_ = [this](...) { ... };
  }
};
```

If scanner data arrives between `SetProcedureStartTime()` and setting the procedure, it will be ignored.

**Impact:** Initial observation might be missed, causing calibration to fail.

### 13. **Missing Dependency Validation**

The code uses `dep_ctx_` members without null checks:
- `dep_ctx_->web_hmi`
- `dep_ctx_->scanner_client`
- `dep_ctx_->activity_status`
- `dep_ctx_->joint_geometry_provider`
- `dep_ctx_->system_clock_now_func`
- `dep_ctx_->calibration_logger`
- `dep_ctx_->kinematics_client`

**Impact:** Potential null pointer dereferences if DependencyContext is not fully initialized.

### 14. **Inconsistent Error Messages**

Error messages vary in format:
- Some include context: `"LW Calibration left touch procedure failed: {}"`
- Some are generic: `"Procedure expired"`
- Some are technical: `"Timeout waiting for scanner data"`

**Impact:** Inconsistent user experience, harder to diagnose issues.

## Comparison Summary

| Aspect | Existing (Circular) | New (Longitudinal) |
|--------|---------------------|-------------------|
| **Observations** | Many (grid-based) | 4 (1 initial + 3 touch) |
| **Automation** | Full (sequence runner) | Manual only |
| **Computation** | Implemented (solver) | Dummy (returns zeros) |
| **Quality Metrics** | Yes (RSE, R², etc.) | No |
| **Validation** | RSE < 1.0 check | None |
| **Model Update** | Yes | No |
| **Progress Reporting** | Yes | No |
| **Error Recovery** | Limited | None |
| **Logging** | Comprehensive | Minimal |
| **Top Center Calc** | Yes | No |

## Recommendations

### Critical (Must Fix)

1. **Implement Calibration Computation**
   - Replace dummy `TryComputeCalibrationResult()` with actual algorithm
   - Or integrate with existing `CalibrationSolver` if applicable
   - Compute actual torch-to-LPCS translation

2. **Add Quality Validation**
   - Compute quality metrics (RSE, residuals)
   - Reject calibrations that don't meet quality thresholds
   - Don't accept zero-vector results

3. **Update Model Configuration**
   - After successful calibration, update `ModelConfig`
   - Ensure calibration is actually applied to the system

4. **Fix Resource Leaks**
   - Ensure `kinematics_client->Release()` is called in all paths
   - Consolidate cleanup logic

### High Priority

5. **Implement StoredLWCalResult**
   - Create storage class if it doesn't exist
   - Define database schema
   - Implement serialization/deserialization

6. **Add Joint Geometry Validation**
   - Implement the TODO: validate `joint_geometry.type == "lw"`
   - Reject wrong geometry types early

7. **Improve Initial Observation**
   - Make confidence requirement configurable (not always HIGH)
   - Add retry mechanism
   - Add timeout with better error message

8. **Consolidate Cleanup**
   - Create `StopCalibration()` method like existing implementation
   - Remove code duplication
   - Ensure consistent cleanup in all paths

### Medium Priority

9. **Add Progress Reporting**
   - Report progress for each step (initial obs, top, left, right)
   - Give user feedback during calibration

10. **Enhance Logging**
    - Log geometric constants
    - Log configuration parameters
    - Log quality metrics when available

11. **Improve Error Messages**
    - Standardize error message format
    - Include more context
    - Make messages user-friendly

12. **Fix Race Condition**
    - Set procedure before starting scanner
    - Or handle early scanner data arrival

### Low Priority

13. **Add Dependency Validation**
    - Validate DependencyContext members are not null
    - Provide clear error messages if dependencies missing

14. **Add Top Center Calculation**
    - If needed for LW calibration, implement calculation
    - Or document why it's not needed

## Conclusion

The `LWCalibrationHandler` is a **simpler but incomplete** implementation compared to the existing circular welding calibration. The most critical issue is that **calibration computation is not implemented** - it always returns zeros, making the calibration useless.

The implementation follows a similar pattern to `CWCalibrationHandler` but has additional issues:
- Missing actual computation logic
- No quality validation
- No model configuration update
- Incomplete error handling
- Resource management issues

Before this can be used in production, the calibration computation must be implemented and the critical issues must be addressed.
