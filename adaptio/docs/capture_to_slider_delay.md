# Capture-to-slider request delay

This note describes how to measure the end-to-end delay from camera capture until a new slider position is requested by the scanner logic.

- Start scanner with metrics enabled.
- For each received image, the Basler upstream handler sets `Image.timestamp` as capture time minus estimated sensor readout delay.
- The scanner stores processed slices in its buffer with processing timestamp.

Procedure:
1. Run scanner for at least 10s in a representative scene with groove known.
2. Scrape Prometheus endpoint and compute:
   - `scanner_image_processing_duration_seconds` (processing time)
   - Time delta between `Image.timestamp` and when `ScannerOutputCB::ScannerOutput` is invoked (requires application-level logging of send time).
3. Report median and 90p values.

Expected results:
- With vertical ROI 500 px and horizontal ROI ~2000 px, end-to-end delay target < 60–80 ms.
