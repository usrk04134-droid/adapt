# Backtrace Analysis: adaptio-wrapped Thread Pool Crash

## Summary

The application crashed with `SIGABRT` in a Boost.Asio thread pool worker thread. The crash occurred due to an unhandled exception that propagated to `std::terminate()`.

## Crash Location

**Thread**: #1 (`.adaptio-wrappe`)  
**Signal**: `SIGABRT`  
**Stack Trace**:
```
frame #0: __pthread_kill_implementation
frame #1: raise
frame #2: abort
frame #3: __gnu_cxx::__verbose_terminate_handler()
frame #4: __cxxabiv1::__terminate(void (*)())
frame #5: std::terminate()
frame #6: boost::asio::detail::posix_thread::func<boost::asio::thread_pool::thread_function>::run()
frame #7: boost_asio_detail_posix_thread_function
frame #8: start_thread
```

## Root Cause Analysis

### The Problem

The crash originates from **line 186** in `src/scanner/scanner_impl.cc`, where work is posted to a Boost.Asio thread pool:

```cpp
void ScannerImpl::ImageGrabbed(std::unique_ptr<image::Image> image) {
  auto sp_image = std::shared_ptr<image::Image>(std::move(image));
  post_([this, image = std::move(sp_image)]() {
    // 135 lines of image processing code (lines 187-321)
    // NO EXCEPTION HANDLING
  });
}
```

The lambda function (lines 186-321) contains extensive image processing logic but **lacks any try-catch blocks**. When an exception is thrown within this lambda:

1. The exception propagates up through the Boost.Asio thread pool infrastructure
2. Since thread pool threads cannot propagate exceptions across thread boundaries
3. The C++ runtime calls `std::terminate()`
4. This results in `SIGABRT` and program termination

### Why This Happens

According to C++ standard behavior:
- Exceptions that escape the top-level function of a thread call `std::terminate()`
- Boost.Asio thread pools run user-provided functions in worker threads
- Any unhandled exception in these functions terminates the entire application

### The Thread Pool Setup

From `scanner_impl.cc` lines 57-61:
```cpp
ScannerImpl::ScannerImpl(...)
    : ...
      m_threadpool(12),  // 12 worker threads
      ... {
  ...
  post_ = [this](std::function<void()> fn) { 
    boost::asio::post(m_threadpool, std::move(fn)); 
  };
}
```

The thread pool has 12 worker threads, and any one of them can crash the entire application if an exception escapes.

## Potential Exception Sources

The `ImageGrabbed` lambda performs many operations that could throw:

1. **Mutex Operations** (lines 194, 199, 223, 225, 230, 270, 282, 291)
   - `m_buffer_mutex.lock()`
   - `m_config_mutex.lock()`

2. **Smart Pointer Operations**
   - Dereferencing potentially null pointers
   - `slice_provider_->GetSlice()` (line 195)
   - `joint_model_->Parse()` (line 201)

3. **Image Processing** (line 201)
   - `joint_model_->Parse()` - Complex computation that could throw
   - Memory allocations for large images
   - Eigen matrix operations

4. **Standard Library Containers**
   - `std::map::at()` calls (lines 273, 295) - throws `std::out_of_range`
   - Vector operations

5. **Logging Operations**
   - Various `LOG_*` calls throughout the function

6. **Prometheus Metrics**
   - Counter and gauge operations (lines 273, 276, 295, 301, 304, 320)

## Evidence

### No Exception Handling Found
Search for try-catch blocks in the file:
```bash
$ rg "catch|try" src/scanner/scanner_impl.cc
# Result: Only matches on "registry" and "entry" - NO try-catch blocks!
```

### The Vulnerable Code Section
The entire lambda (lines 186-321) processes images without any exception safety:
- 135 lines of code
- Multiple potential exception sources
- Zero exception handlers

## Recommendations

### 1. **Add Exception Handling to ImageGrabbed Lambda** (Critical)

Wrap the entire lambda body in a try-catch block:

```cpp
void ScannerImpl::ImageGrabbed(std::unique_ptr<image::Image> image) {
  auto sp_image = std::shared_ptr<image::Image>(std::move(image));
  post_([this, image = std::move(sp_image)]() {
    try {
      // ... existing code (lines 190-321)
    } catch (const std::exception& e) {
      LOG_ERROR("Exception in image processing thread: {}", e.what());
      metrics_.image_consecutive_errors->Increment(1);
      // Optionally: notify error handler, set error state, etc.
    } catch (...) {
      LOG_ERROR("Unknown exception in image processing thread");
      metrics_.image_consecutive_errors->Increment(1);
    }
  });
}
```

### 2. **Review All Thread Pool Usage** (Important)

Search for all uses of `boost::asio::post` and ensure exception handling:
```bash
$ rg "boost::asio::post|asio.*post"
```

### 3. **Add Defensive Checks** (Recommended)

Before dereferencing pointers in the lambda:
```cpp
if (!slice_provider_ || !joint_model_ || !image_logger_) {
  LOG_ERROR("Null pointer in image processing");
  return;
}
```

### 4. **Use Safe Container Access** (Recommended)

Replace `std::map::at()` with `find()` or check `contains()` first:
```cpp
// Instead of:
metrics_.image.at(slice.num_walls_found)->Increment();

// Use:
if (auto it = metrics_.image.find(slice.num_walls_found); it != metrics_.image.end()) {
  it->second->Increment();
}
```

### 5. **Consider Thread Pool Join Safety** (Important)

The `Stop()` method (line 165) calls `m_threadpool.join()` which waits for all work to complete. Consider:
- Adding a timeout
- Handling exceptions during join
- Ensuring proper cleanup order

### 6. **Add Thread Pool Exception Handler** (Advanced)

Boost.Asio supports custom exception handlers for thread pools:
```cpp
// In constructor:
m_threadpool.get_executor().context().set_exception_handler(
  [](std::exception_ptr e) {
    try {
      if (e) std::rethrow_exception(e);
    } catch (const std::exception& ex) {
      LOG_CRITICAL("Unhandled exception in thread pool: {}", ex.what());
    }
  }
);
```

## Testing Recommendations

1. **Inject Faults**: Test with deliberately failing operations
2. **Load Testing**: Process many images rapidly to expose race conditions
3. **Memory Testing**: Run under valgrind/sanitizers
4. **Exception Injection**: Use mocks to force exceptions in critical paths

## Related Files

- `src/scanner/scanner_impl.h` - ScannerImpl class definition
- `src/scanner/scanner_impl.cc` - Implementation with vulnerable code
- `src/scanner/joint_model/joint_model.h` - Parse() method definition
- `src/scanner/slice_provider/slice_provider.h` - GetSlice() method

## Conclusion

The crash is caused by an unhandled exception in the image processing thread pool. The fix is straightforward: add proper exception handling to the lambda function in `ImageGrabbed()`. This is a critical issue that can cause unpredictable crashes during image processing operations.

**Priority**: **CRITICAL** - Can cause application crashes during normal operation

**Effort**: **Low** - Simple try-catch wrapper needed

**Risk**: **Low** - Exception handling is a safe addition that won't break existing functionality
