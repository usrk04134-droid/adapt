# Calibration Logic Comparison: Existing vs New Implementation

## Overview

This document compares the existing `CalibrationManagerImpl` implementation with the new `CWCalibrationHandler` implementation for longitudinal welding calibration.

## Key Architectural Differences

### 1. **Dependency Injection Pattern**

**Existing (`CalibrationManagerImpl`):**
- Uses individual pointer parameters for each dependency
- Constructor takes 13+ individual parameters:
  ```cpp
  CalibrationManagerImpl(
      SQLite::Database* db, 
      zevs::Timer* timer, 
      scanner_client::ScannerClient* scanner_client,
      CalibrationSolver* calibration_solver,
      // ... many more individual parameters
  )
  ```

**New (`CWCalibrationHandler`):**
- Uses a `DependencyContext*` to group dependencies
- Constructor takes fewer parameters:
  ```cpp
  CWCalibrationHandler(
      SQLite::Database* db, 
      DependencyContext* dep_ctx,
      CalibrationSolver* calibration_solver,
      // ... fewer parameters
  )
  ```

**Impact:** The new approach reduces constructor parameter count but introduces a dependency on `DependencyContext` structure.

### 2. **Activity Status Handling**

**Existing:**
- Only checks for `WELD_OBJECT_CALIBRATION` status
- Simpler state management

**New:**
- Checks for both `WELD_OBJECT_CALIBRATION` and `CALIBRATION_AUTO_MOVE`
- Also checks for `LW_CALIBRATION` in `OnLaserTorchCalSet`:
  ```cpp
  if (dep_ctx_->activity_status->Get() == coordination::ActivityStatusE::WELD_OBJECT_CALIBRATION ||
      dep_ctx_->activity_status->Get() == coordination::ActivityStatusE::LW_CALIBRATION) {
  ```

**Impact:** New implementation supports multiple calibration types but adds complexity.

### 3. **Calibration Result Storage**

**Existing:**
```cpp
auto calibration_result = StoredCalibrationResult::FromCalibrationResult(
    result.value(), calibration_ctx_.weld_object_radius);
```

**New:**
```cpp
auto calibration_result = StoredCalibrationResult::FromCalibrationResult(
    result.value(), cal_ctx_.weld_object_radius, 
    cal_ctx_.wire_diameter, cal_ctx_.stickout);
```

**Critical Issue:** The new code calls `FromCalibrationResult` with 4 parameters, but the existing `StoredCalibrationResult::FromCalibrationResult` only accepts 2 parameters. This will cause a **compilation error** unless the API has been updated.

### 4. **Cleanup Method**

**Existing:**
- Uses `StopCalibration()` method that:
  - Stops scanner
  - Clears context
  - Resets procedures
  - Sets activity status to IDLE

**New:**
- Uses `Cleanup()` method that:
  - Only clears context and procedures
  - Does NOT stop scanner or set activity status
  - Requires explicit calls to stop scanner and set status elsewhere

**Impact:** New approach is less encapsulated - cleanup logic is scattered across multiple locations.

### 5. **Busy State Check**

**Existing:**
```cpp
auto Busy() const -> bool {
  return calibration_start_procedure_.has_value() || 
         calibration_top_pos_procedure_.has_value() ||
         calibration_left_pos_procedure_.has_value() || 
         calibration_right_pos_procedure_.has_value() ||
         sequence_runner_;  // Checks if pointer exists
}
```

**New:**
```cpp
auto IsBusy() const -> bool {
  return calibration_start_procedure_.has_value() || 
         calibration_top_pos_procedure_.has_value() ||
         calibration_left_pos_procedure_.has_value() || 
         calibration_right_pos_procedure_.has_value() ||
         (sequence_runner_ && sequence_runner_->Busy());  // Checks if exists AND busy
}
```

**Impact:** New implementation is more precise - checks if sequence runner is actually busy, not just if it exists.

### 6. **Error Handling in OnLaserTorchCalSet**

**Existing:**
```cpp
if (activity_status_->Get() == coordination::ActivityStatusE::WELD_OBJECT_CALIBRATION) {
  StopCalibration();
}
```

**New:**
```cpp
if (dep_ctx_->activity_status->Get() == coordination::ActivityStatusE::WELD_OBJECT_CALIBRATION ||
    dep_ctx_->activity_status->Get() == coordination::ActivityStatusE::LW_CALIBRATION) {
  dep_ctx_->scanner_client->Stop();
  dep_ctx_->activity_status->Set(coordination::ActivityStatusE::IDLE);
}
```

**Impact:** New implementation handles more states but duplicates cleanup logic instead of using a centralized method.

## Drawbacks of New Implementation

### 1. **API Mismatch - Compilation Error**
The most critical issue: `StoredCalibrationResult::FromCalibrationResult` is called with 4 parameters but only accepts 2. This will cause a compilation failure unless:
- The `StoredCalibrationResult` class is updated to accept additional parameters
- Or the call site is corrected to match the existing API

### 2. **Inconsistent Cleanup Logic**
- `Cleanup()` method doesn't fully clean up (doesn't stop scanner or reset activity status)
- Cleanup logic is duplicated in multiple places:
  - `HandleTopTouchFailure`
  - `HandleLeftTouchFailure`
  - `HandleRightTouchFailure`
  - `OnCalibrationSequenceComplete`
  - `OnWeldObjectCalStop`
  
This violates DRY (Don't Repeat Yourself) principle and makes maintenance harder.

### 3. **Tight Coupling to DependencyContext**
- Requires a `DependencyContext` structure that may not exist
- Makes the code less flexible - can't easily swap individual dependencies
- If `DependencyContext` changes, all dependent code breaks

### 4. **Missing Metrics Tracking**
The existing implementation has:
```cpp
std::unique_ptr<CalibrationMetrics> cal_metrics_;
```
But the new implementation doesn't appear to track calibration metrics. This could impact monitoring and analytics.

### 5. **Potential Memory/Resource Leaks**
The new `Cleanup()` method doesn't explicitly release the `kinematics_client_` in all error paths. The existing implementation calls `kinematics_client_->Release()` in failure callbacks, but the new implementation may miss some paths.

### 6. **Less Defensive Programming**
The existing implementation has more explicit state checks and validation. The new implementation relies more on the `DependencyContext` being properly initialized, which could lead to null pointer dereferences if not careful.

### 7. **Incomplete Interface Implementation**
The existing `CalibrationManagerImpl` implements:
- `scanner_client::ScannerObserver` interface
- `coordination::CalibrationStatus` interface

The new `CWCalibrationHandler` may not implement these interfaces, which could break integration with other components.

### 8. **Activity Status Management**
The new implementation checks for `LW_CALIBRATION` status but it's unclear if this is a new status type or if it should be handled differently. This could lead to:
- Confusion about which calibration type is active
- Potential state machine conflicts
- Harder debugging

## Recommendations

1. **Fix API Mismatch**: Either update `StoredCalibrationResult::FromCalibrationResult` to accept wire_diameter and stickout, or remove those parameters from the call.

2. **Consolidate Cleanup**: Create a single `StopCalibration()` method that handles all cleanup (scanner stop, status reset, context clear, resource release).

3. **Add Metrics**: Include calibration metrics tracking if it's needed for monitoring.

4. **Verify DependencyContext**: Ensure `DependencyContext` structure exists and is properly defined before using this implementation.

5. **Interface Compliance**: Ensure `CWCalibrationHandler` implements the same interfaces as `CalibrationManagerImpl` for compatibility.

6. **Error Path Coverage**: Review all error paths to ensure `kinematics_client_->Release()` is called consistently.

7. **State Machine Clarity**: Document the relationship between `WELD_OBJECT_CALIBRATION`, `LW_CALIBRATION`, and `CALIBRATION_AUTO_MOVE` states.

## Summary

The new implementation attempts to simplify dependency management through `DependencyContext` and adds support for additional calibration types. However, it introduces several issues:

- **Critical**: API mismatch that will prevent compilation
- **Major**: Inconsistent cleanup logic that could lead to resource leaks
- **Moderate**: Missing metrics and interface implementations
- **Minor**: Reduced flexibility due to tight coupling

The existing implementation is more mature, has better error handling, and follows better encapsulation principles. The new implementation would benefit from addressing these issues before replacing the existing one.
