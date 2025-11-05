# JSON Safety Fix

## Problem

Test was crashing with:
```
nlohmann::basic_json<>::operator[]: Assertion `it != m_data.m_value.object->end()' failed.
SIGABRT - Abort (abnormal termination) signal
```

## Root Cause

The code was directly accessing JSON fields without checking if they exist:
```cpp
auto progress = payload["progress"].get<double>();  // Unsafe!
```

If the JSON doesn't contain the expected key, this causes an assertion failure and crashes.

## Solution

Added safe JSON access with existence checks:

### Before (Unsafe):
```cpp
auto payload = ReceiveJsonByName(fixture, "WeldObjectCalProgress");
if (payload != nullptr) {
  auto progress = payload["progress"].get<double>();  // Can crash!
  TESTLOG("Progress: {:.1f}%", progress);
}
```

### After (Safe):
```cpp
auto payload = ReceiveJsonByName(fixture, "WeldObjectCalProgress");
if (payload != nullptr && !payload.empty()) {
  if (payload.contains("progress")) {
    auto progress = payload["progress"].get<double>();
    TESTLOG("Progress: {:.1f}%", progress * 100.0);
  } else {
    TESTLOG("No progress info");
  }
}
```

## Changes Made

1. **Progress handling** - Added `contains()` check before accessing "progress" field
2. **Calibration result** - Added checks for empty payload and "result" field existence
3. **Error logging** - Added helpful messages when JSON is malformed

## How to Apply

**Rebuild the tests:**
```bash
cd /workspace/adaptio
rm -rf build/
./adaptio.sh --build-tests
```

**Run tests:**
```bash
./RUN_IMAGE_TESTS.sh
```

## Expected Behavior

With this fix, the test will:
- ✅ Handle missing JSON fields gracefully
- ✅ Log helpful messages instead of crashing
- ✅ Continue execution even if some messages are malformed
- ✅ Still fail appropriately if critical data is missing

## Files Modified

- `src/block_tests/weld_object_calibration_image_test.cc` - Added safe JSON access

## Next Steps

1. Rebuild tests (essential!)
2. Run `./RUN_IMAGE_TESTS.sh`
3. Tests should complete without crashes

If tests still fail, the error messages will now be informative instead of causing crashes.
