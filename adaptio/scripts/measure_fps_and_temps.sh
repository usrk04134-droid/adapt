#!/usr/bin/env bash
set -euo pipefail

# Simple placeholder script to query Prometheus metrics from Adaptio logs or endpoint
# Usage: ./measure_fps_and_temps.sh

# Adjust URL to your metrics endpoint if exposed, otherwise tail logs and parse
METRICS_URL="http://localhost:9100/metrics"

if command -v curl >/dev/null 2>&1; then
  echo "# Fetching selected metrics from ${METRICS_URL}"
  curl -s "$METRICS_URL" | grep -E "^(scanner_input_fps|scanner_capture_to_slider_delay_ms|basler_camera_temperature|basler_camera_max_temperature)" || true
else
  echo "curl not available; please query metrics via your monitoring stack."
fi
