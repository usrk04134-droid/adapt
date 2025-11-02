# Analysis: When `coordinates.cols()` Returns Negative Values

## Overview

In Eigen, `cols()` returns `Eigen::Index` which is typically `std::ptrdiff_t` (a signed integer type). While Eigen matrices should never have negative dimensions under normal circumstances, there are several edge cases and potential issues that could lead to negative values being returned.

## Root Causes

### 1. **Uninitialized Matrix Objects**
If a `WorkspaceCoordinates` or `PlaneCoordinates` matrix is created but never properly initialized, accessing `cols()` could return garbage memory values, which could appear as negative.

**Example scenario:**
```cpp
WorkspaceCoordinates coordinates;  // Default constructor - uninitialized
auto cols = coordinates.cols();    // Could return garbage/negative value
```

**In the codebase:**
- Line 67 in `tilted_perspective_camera.cc`: `PlaneCoordinates camera_coordinates = PlaneCoordinates::Zero(2, workspace_coordinates.cols());`
  - If `workspace_coordinates` is uninitialized or corrupted, `workspace_coordinates.cols()` could be negative
  - This would then create `camera_coordinates` with invalid dimensions

### 2. **Memory Corruption**
If the matrix's internal memory is corrupted (e.g., buffer overflows, use-after-free, stack corruption), the internal size fields could be overwritten with negative values.

**Vulnerable locations:**
- Matrix operations that involve pointer arithmetic
- Operations with arrays from external data sources
- Stack-based matrix allocations that get corrupted

### 3. **Integer Overflow/Underflow**
While unlikely for typical matrix sizes, if `cols()` is extremely large and operations cause integer overflow, it could wrap around to negative values:

```cpp
Eigen::Index large_cols = std::numeric_limits<Eigen::Index>::max();
// Some operation causes overflow
Eigen::Index result = large_cols + 1;  // Could wrap to negative
```

### 4. **Invalid Matrix Operations**
Certain invalid operations could result in matrices with corrupted state:

**Line 68 in `tilted_perspective_camera.cc`:**
```cpp
camera_coordinates << workspace_coordinates.row(0).array(), workspace_coordinates.row(1).array();
```

If `workspace_coordinates` has invalid dimensions or is corrupted:
- The `row(0).array()` and `row(1).array()` expressions might produce invalid results
- The `<<` operator might fail to properly set dimensions
- The resulting `camera_coordinates` could have invalid state

### 5. **Eigen Array Expression Issues**
When using Eigen's array expressions (`.array()`), the resulting expressions might not preserve size information correctly in edge cases:

**In `Undistort` function (camera_model.cc:88-111):**
```cpp
const RowVectorXd x_undistorted = distorted.row(0).array() * (K1 * rd2.array() + ...);
const RowVectorXd y_undistorted = distorted.row(1).array() * (K1 * rd2.array() + ...);
PlaneCoordinates undistorted(2, x_undistorted.cols());
```

If `distorted` has corrupted state:
- `distorted.row(0).array()` could have invalid size
- Array operations might propagate invalid sizes
- `x_undistorted.cols()` could be negative

### 6. **Zero-Sized Matrix Edge Cases**
While zero-sized matrices are valid in Eigen, some operations might not handle them correctly:

```cpp
PlaneCoordinates empty(2, 0);  // Valid: 2x0 matrix
auto cols = empty.cols();      // Returns 0, not negative
```

However, if a zero-sized matrix is then used in operations that expect non-zero sizes, the results could be unpredictable.

## Specific Code Locations to Check

### Location 1: `ImageToWorkspace` - Line 67-68
```cpp
PlaneCoordinates camera_coordinates = PlaneCoordinates::Zero(2, workspace_coordinates.cols());
camera_coordinates << workspace_coordinates.row(0).array(), workspace_coordinates.row(1).array();
```

**Potential issue:** If `workspace_coordinates.cols()` is negative, `PlaneCoordinates::Zero()` might create an invalid matrix, or Eigen might reject it with an assertion. The `<<` operation could also fail if dimensions don't match.

### Location 2: `ImageToWorkspace` - Line 77
```cpp
WorkspaceCoordinates wcs_coordinates = WorkspaceCoordinates::Zero(3, camera_coordinates.cols());
```

**Potential issue:** If `camera_coordinates.cols()` is negative (from previous step), this creates another invalid matrix.

### Location 3: `Undistort` - Line 107
```cpp
PlaneCoordinates undistorted(2, x_undistorted.cols());
```

**Potential issue:** If array operations on `distorted` produce invalid `RowVectorXd` objects, `x_undistorted.cols()` could be negative.

### Location 4: `ImagePlaneToLaserPlane` - Line 46-47
```cpp
auto z_component = RowVectorXd::Zero(image_coordinates.cols()).array() + w * focus_distance;
Coordinates coordinates(image_coordinates.rows() + z_component.rows(), z_component.cols());
```

**Potential issue:** If `image_coordinates.cols()` is negative or invalid, `z_component` will have invalid dimensions, propagating the error.

## Diagnostic Recommendations

1. **Add validation checks:**
   ```cpp
   if (workspace_coordinates.cols() < 0) {
       LOG_ERROR("Invalid workspace_coordinates.cols() = {}", workspace_coordinates.cols());
       // Handle error
   }
   ```

2. **Check matrix initialization:**
   - Ensure all matrices are properly initialized before use
   - Validate input matrices from external sources

3. **Use Eigen assertions:**
   - Compile with `EIGEN_INITIALIZE_MATRICES_BY_ZERO` to catch uninitialized matrices
   - Enable Eigen's built-in assertions (`EIGEN_NO_DEBUG` not defined)

4. **Add defensive checks:**
   ```cpp
   const Eigen::Index cols = workspace_coordinates.cols();
   if (cols < 0 || cols > MAX_EXPECTED_COLS) {
       return boost::outcome_v2::failure(/* error code */);
   }
   ```

5. **Trace matrix dimensions through the pipeline:**
   - Log `cols()` values at each step to identify where negative values first appear
   - Check the input `image_coordinates` to ensure it's valid

## Conclusion

Negative `cols()` values indicate a serious bug - either:
- **Uninitialized memory** (matrix not properly constructed)
- **Memory corruption** (buffer overflows, use-after-free)
- **Invalid operations** (corrupted matrix state from previous operations)
- **Integer overflow** (extremely unlikely for typical matrix sizes)

The most likely cause is **uninitialized or corrupted matrix objects** being passed through the transformation pipeline. The issue likely originates from the input `image_coordinates` parameter or from matrices created during intermediate steps that aren't properly initialized.
