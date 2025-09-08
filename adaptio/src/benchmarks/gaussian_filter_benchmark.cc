
#include <fmt/core.h>

#include <fstream>

#include "../common/filters/gaussian_filter.h"
#include "test_data.h"

auto const PRINT_TO_FILE = false;

void GaussianFilterTest() {
  {
    auto filter = common::filters::GaussianFilter(301, 50);

    std::ofstream output_file;
    if (PRINT_TO_FILE) {
      output_file = std::ofstream("gaussian_filter_test1.log");
    }

    for (auto value : DATA_WELD_SPEED) {
      auto filtered_value = 0.0;
      for (auto i = 0; i < 20; ++i) {
        filtered_value = filter.Update(value);
      }

      if (PRINT_TO_FILE) {
        output_file << fmt::format("{}\n", value, filtered_value);
      }
      fmt::println("orig: {} filtered: {}", value, filtered_value);
    }

    if (PRINT_TO_FILE) {
      output_file.close();
    }
  }

  {
    auto filter = common::filters::GaussianFilter(301, 50);

    std::ofstream output_file;
    if (PRINT_TO_FILE) {
      output_file = std::ofstream("gaussian_filter_test2.log");
    }

    for (auto value : DATA_WS_CURRENT) {
      auto filtered_value = 0.0;
      for (auto i = 0; i < 20; ++i) {
        filtered_value = filter.Update(value);
      }

      if (PRINT_TO_FILE) {
        output_file << fmt::format("{} {}\n", value, filtered_value);
      }
      fmt::println("orig: {} filtered: {}", value, filtered_value);
    }

    if (PRINT_TO_FILE) {
      output_file.close();
    }
  }
}
