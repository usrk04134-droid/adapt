#include "common/filters/gaussian_filter.h"

#include <doctest/doctest.h>

// NOLINTBEGIN(*-magic-numbers)

namespace common::filters {

TEST_SUITE("GaussianFilter") {
  TEST_CASE("BasicTestLargeSigma") {
    auto const kernel_size = 15;
    auto const sigma       = 5;
    auto filter            = GaussianFilter(kernel_size, sigma);

    auto const val_normal = 3.0;
    auto const val_max    = 5.0;
    for (auto i = 0; i < kernel_size; ++i) {
      /* filter not full - return input value */
      CHECK(filter.Update(val_normal) == doctest::Approx(val_normal));
    }

    /* insert outlier */
    CHECK(filter.Update(val_normal * 5) < val_max);

    for (auto i = 0; i < 15; ++i) {
      /* filter not full - return input value */
      CHECK(filter.Update(val_normal) < val_max);
    }
  }
  TEST_CASE("BasicTestSmallSigma") {
    auto const kernel_size = 15;
    auto const sigma       = 1;
    auto filter            = GaussianFilter(kernel_size, sigma);

    auto const val_normal = 3.0;
    auto const val_max    = 8.0;
    for (auto i = 0; i < kernel_size; ++i) {
      /* filter not full - return input value */
      CHECK(filter.Update(val_normal) == doctest::Approx(val_normal));
    }

    /* insert outlier */
    CHECK(filter.Update(val_normal * 5) < val_max);

    for (auto i = 0; i < 15; ++i) {
      /* filter not full - return input value */
      CHECK(filter.Update(val_normal) < val_max);
    }
  }
}

}  // namespace common::filters

// NOLINTEND(*-magic-numbers)
