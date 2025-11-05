# Weld Object Calibration Image Block Test

## Overview

This block test performs weld object calibration using **multiple** real scanner images instead of simulated data. It validates the calibration workflow using actual TIFF images captured from the scanner system. The test is **data-driven** and automatically runs against all configured test images.

## Test File

- **Location**: `src/block_tests/weld_object_calibration_image_test.cc`
- **Test Suite**: `WeldObjectCalibrationImage`

## Test Cases

### 1. `calibrate_from_multiple_images`

The main test case that performs complete weld object calibration using all configured scanner images.

**Key Features:**
- **Data-driven approach**: Automatically tests all images in the `TEST_IMAGES` configuration
- Uses doctest SUBCASE feature to create separate subtests for each image
- Each image can have custom calibration parameters (diameter, stickout, wire diameter, mount angle)
- Extracts groove coordinates using the scanner's Snake algorithm
- Simulates the complete calibration workflow:
  - Laser-to-torch calibration (LTC) setup
  - Weld object calibration start
  - Left and right wall touch point simulation
  - Automatic grid measurement sequence
  - Calibration result validation

**Default Parameters:**
- Weld object diameter: 4.0m (typical pipe)
- Stickout: 25mm
- Wire diameter: 1.2mm
- Scanner mount angle: 0.26 radians (~15 degrees)

**Current Test Images:**
1. `tests/configs/sil/calibration/1738232679592.tiff` - Standard calibration image (3500x500)
2. `tests/configs/sil/1738243625597.tiff` - Alternative calibration image (3500x520)

### 2. `image_loading_test_all`

Verification test that checks all configured images:
- Each image file can be loaded successfully
- ImageBuilder can process each image
- Snake extraction works correctly
- Sufficient groove points are extracted (≥7 points)

### 3. `count_test_images`

Simple utility test that:
- Verifies test images are configured
- Lists all configured test images with descriptions
- Useful for debugging test configuration

## How It Works

### 1. Image Processing

```cpp
ExtractGrooveFromImage(image_path, timestamp)
```

This function:
1. Loads the TIFF image using OpenCV
2. Converts it to a scanner Image using ImageBuilder
3. Extracts the groove profile using the Snake algorithm
4. Converts the snake points to SliceData format (7 groove points)

### 2. Calibration Workflow

```cpp
CalibrateWithImage(fixture, image_path)
```

This function simulates the complete calibration procedure:
1. Sets up laser-to-torch calibration parameters
2. Starts weld object calibration
3. Simulates left wall touch using extracted groove data
4. Simulates right wall touch using extracted groove data
5. Processes automatic grid measurements
6. Validates the calibration result
7. Applies the calibration

### 3. Data Flow

```
TIFF Image
    ↓
OpenCV imread()
    ↓
ImageBuilder
    ↓
Snake::FromImage()
    ↓
SliceData (7 groove points)
    ↓
Calibration Algorithm
    ↓
Calibration Result
```

## Test Images

The test uses multiple real scanner calibration images. Each image is configured via the `TEST_IMAGES` vector with:
- Image path
- Description
- Optional custom calibration parameters

**Currently Configured Images:**

| Image | Path (relative to build dir) | Size | Description |
|-------|------------------------------|------|-------------|
| 1 | `../tests/configs/sil/calibration/1738232679592.tiff` | 3500x500 | Standard calibration image |
| 2 | `../tests/configs/sil/1738243625597.tiff` | 3500x520 | Alternative calibration image |

**Image Format:**
- Format: TIFF, 8-bit grayscale, LZW compression
- Content: Scanner profile of welding grooves

**Note**: Image paths are relative to the build directory (`build/debug/`) where tests execute.

## Adding New Test Images

To add a new test image, update the `TEST_IMAGES` vector in `weld_object_calibration_image_test.cc`:

```cpp
const std::vector<TestImageConfig> TEST_IMAGES = {
    // Existing images...
    
    // Add new image with default parameters
    // NOTE: Path is relative to build directory (build/debug/)
    TestImageConfig(
        "../tests/configs/sil/your_image.tiff",
        "Description of your test image"
    ),
    
    // Or add with custom parameters
    TestImageConfig(
        "../tests/configs/sil/special_image.tiff",
        "Special test case",
        5.0,    // weld_object_diameter_m
        30e-3,  // stickout_m
        1.6,    // wire_diameter_mm
        0.3     // scanner_mount_angle_rad
    ),
};
```

The test will automatically pick up and run against all configured images.

## Usage

### Build the tests

```bash
cd /workspace/adaptio
./adaptio.sh --build-tests
```

### Run all calibration image tests

**IMPORTANT**: Tests must be run from the **build directory** for image paths to resolve correctly.

```bash
cd /workspace/adaptio/build/debug
./src/adaptio-block-tests --doctest-test-suite=WeldObjectCalibrationImage
```

Or from the repository root:
```bash
cd /workspace/adaptio
./build/debug/src/adaptio-block-tests --doctest-test-suite=WeldObjectCalibrationImage
```

This will run all test cases including all configured images (currently 2 images × multiple test cases).

### Run specific test case

```bash
# Run the main calibration test with all images
./build/debug/src/adaptio-block-tests --doctest-test-case=calibrate_from_multiple_images

# Run just the image loading tests
./build/debug/src/adaptio-block-tests --doctest-test-case=image_loading_test_all

# Check configured images
./build/debug/src/adaptio-block-tests --doctest-test-case=count_test_images
```

### Run specific image subtest

```bash
# Run calibration for a specific image by its description
./build/debug/src/adaptio-block-tests --doctest-test-case=calibrate_from_multiple_images \
    --doctest-subcase="Standard calibration image*"
```

### Run with trace output

```bash
./build/debug/src/adaptio-block-tests --trace --doctest-test-suite=WeldObjectCalibrationImage
```

## Dependencies

### External Libraries
- OpenCV (image loading)
- doctest (test framework)
- fmt (formatting)
- nlohmann/json (JSON handling)

### Internal Components
- Scanner image processing (`scanner/image/`)
- Snake algorithm (`scanner/joint_model/snake.h`)
- Calibration helpers (`block_tests/helpers_calibration_v2.h`)
- Test fixtures (`block_tests/helpers.h`)

## Key Differences from Simulator-Based Tests

| Aspect | Simulator Tests | Image-Based Test |
|--------|----------------|------------------|
| Data Source | Deposition simulator | Real TIFF images |
| Groove Detection | Simulated ABW points | Snake extraction from image |
| Torch Movement | Simulated 3D positioning | Simulated 2D positioning based on image data |
| Use Case | Development & validation | Real-world data validation |

## Expected Behavior

When the test runs successfully:
1. All configured images load successfully
2. Snake extraction succeeds for each image (≥7 groove points)
3. Calibration workflow completes without errors for each image
4. Each calibration result contains valid transformation data
5. Results can be applied to the system

**Console Output Example:**
```
Running calibration test for: Standard calibration image - 3500x500
Image path: ./tests/configs/sil/calibration/1738232679592.tiff
Loaded image: ... (3500x500)
Extracted groove with 7 points from snake of size 1847
Testing image: Standard calibration image - 3500x500
Parameters: diameter=4.0m, stickout=25.0mm, wire=1.2mm, mount_angle=0.260rad
...
[doctest] test cases:  3 |  3 passed | 0 failed | 0 skipped
[doctest] assertions: 14 | 14 passed | 0 failed |
```

## Troubleshooting

### Image not found / can't open/read file
**Error**: `Failed to load image: ../tests/configs/sil/calibration/1738232679592.tiff`

**Causes and Solutions**:
1. **Wrong working directory**
   - Tests must run from `build/debug/` directory
   - Solution: `cd build/debug` before running tests
   
2. **Image file missing**
   - Verify file exists: `ls ../tests/configs/sil/calibration/*.tiff` (from build/debug/)
   - Copy image files to correct location if missing

3. **Incorrect paths in TEST_IMAGES**
   - Paths must be relative to build directory
   - Use `../tests/...` format, not `./tests/...`

### Snake extraction fails
- Verify image contains visible groove profile
- Check image threshold parameter (currently set to 16)
- Ensure image is grayscale format

### Calibration fails
- Check if extracted groove has at least 7 points
- Verify kinematics and scanner mocks are properly set up
- Enable TESTLOG by uncommenting `#define TESTLOG_DISABLED` at the top of the file

## Features

**Current Features:**
- ✅ Multiple test images support
- ✅ Data-driven test configuration
- ✅ Custom parameters per image
- ✅ Automatic iteration through all images
- ✅ Individual subtest reporting per image

**Future Enhancements:**

Potential improvements for this test:
1. Add more test images for different groove types (V-groove, U-bevel, etc.)
2. Compare results with simulator-based calibration
3. Add validation of calibration accuracy metrics
4. Test error handling for corrupted/invalid images
5. Performance benchmarking across different images
6. Add test images with known expected calibration results for validation

## Related Files

- `calibration_v2_test.cc` - Simulator-based calibration tests
- `helpers_calibration_v2.h` - Calibration helper functions
- `scanner/joint_model/snake.h` - Snake extraction algorithm
- `scanner/image/image_builder.h` - Image processing utilities

## References

- Calibration v2 documentation: `docs/adaptio.md`
- Block tests documentation: `src/block_tests/BLOCK_TESTS.md`
- Scanner documentation: `docs/scanner/scanner.md`
