# Weld Object Calibration Image Block Test

## Overview

This block test performs weld object calibration using real scanner images instead of simulated data. It validates the calibration workflow using actual TIFF images captured from the scanner system.

## Test File

- **Location**: `src/block_tests/weld_object_calibration_image_test.cc`
- **Test Suite**: `WeldObjectCalibrationImage`

## Test Cases

### 1. `calibrate_from_real_image`

The main test case that performs a complete weld object calibration using a real scanner image.

**Key Features:**
- Loads a TIFF image from `tests/configs/sil/calibration/1738232679592.tiff`
- Extracts groove coordinates using the scanner's Snake algorithm
- Simulates the calibration workflow including:
  - Laser-to-torch calibration (LTC) setup
  - Weld object calibration start
  - Left and right wall touch point simulation
  - Automatic grid measurement sequence
  - Calibration result validation

**Parameters:**
- Weld object diameter: 4.0m (typical pipe)
- Stickout: 25mm
- Wire diameter: 1.2mm
- Scanner mount angle: 0.26 radians (~15 degrees)

### 2. `image_loading_test`

A simple verification test that checks:
- Image file can be loaded successfully
- ImageBuilder can process the image
- Snake extraction works correctly

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

## Test Image

The test uses a real scanner calibration image:
- **Path**: `tests/configs/sil/calibration/1738232679592.tiff`
- **Format**: TIFF, 3500x500 pixels, 8-bit grayscale, LZW compression
- **Content**: Scanner profile of a welding groove

## Usage

### Build the tests

```bash
cd /workspace/adaptio
./adaptio.sh --build-tests
```

### Run all calibration image tests

```bash
./build/debug/src/adaptio-block-tests --doctest-test-suite=WeldObjectCalibrationImage
```

### Run specific test case

```bash
./build/debug/src/adaptio-block-tests --doctest-test-case=calibrate_from_real_image
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
1. Image loads and snake extraction succeeds
2. Calibration workflow completes without errors
3. Calibration result contains valid transformation data
4. Result can be applied to the system

## Troubleshooting

### Image not found
- Ensure the test image exists at `tests/configs/sil/calibration/1738232679592.tiff`
- Check the working directory when running tests

### Snake extraction fails
- Verify image contains visible groove profile
- Check image threshold parameter (currently set to 16)
- Ensure image is grayscale format

### Calibration fails
- Check if extracted groove has at least 7 points
- Verify kinematics and scanner mocks are properly set up
- Enable TESTLOG by uncommenting `#define TESTLOG_DISABLED` at the top of the file

## Future Enhancements

Potential improvements for this test:
1. Support multiple test images for different groove types
2. Add parametric testing with different calibration parameters
3. Compare results with simulator-based calibration
4. Add validation of calibration accuracy metrics
5. Test error handling for corrupted/invalid images

## Related Files

- `calibration_v2_test.cc` - Simulator-based calibration tests
- `helpers_calibration_v2.h` - Calibration helper functions
- `scanner/joint_model/snake.h` - Snake extraction algorithm
- `scanner/image/image_builder.h` - Image processing utilities

## References

- Calibration v2 documentation: `docs/adaptio.md`
- Block tests documentation: `src/block_tests/BLOCK_TESTS.md`
- Scanner documentation: `docs/scanner/scanner.md`
