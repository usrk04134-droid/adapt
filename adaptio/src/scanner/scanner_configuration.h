#pragma once

#include <cstdint>

namespace scanner {

struct ScannerConfigurationData {
  int64_t gray_minimum_top;
  int64_t gray_minimum_wall;
  int64_t gray_minimum_bottom;
  // Horizontal ROI tuning
  int32_t roi_margin_px        = 300;
  int32_t roi_min_width_px     = 800;
  int32_t roi_step_px          = 100;
  int32_t roi_hysteresis_px    = 50;
};

}  // namespace scanner
