#pragma once

#include <cstdint>
#include <memory>

namespace core::scanner {

struct ScannerConfigurationData {
  int64_t gray_minimum_top;
  int64_t gray_minimum_wall;
  int64_t gray_minimum_bottom;
};

}  // namespace core::scanner
