#pragma once

#include <cstdint>

namespace scanner {

struct ScannerConfigurationData {
  int64_t gray_minimum_top;
  int64_t gray_minimum_wall;
  int64_t gray_minimum_bottom;
};

}  // namespace scanner
