#include "common/math/filter.h"

#include <doctest/doctest.h>

#include <Eigen/Core>

using Eigen::Index;
using Eigen::RowVectorX;
using Eigen::RowVectorXd;
using Eigen::Vector3d;

// NOLINTBEGIN(*-magic-numbers)
namespace common::math::filter {
TEST_SUITE("Filtering") {
  TEST_CASE("Median filter") {
    RowVectorXd source(15);
    source << 1, 5, 23, 4, 9, 12, 19, 32, 200, 12, 1, 2, 3, 4, 5;
    // 1: not enough context
    // 5: not enough context
    // 23: 1 4 [5] 9 23
    // 4: 4 5 [9] 12 23
    // 9: 4 9 [12] 19 23
    // 12: 4 9 [12] 19 32
    // 19: 9 12 [19] 32 200
    // 32: 12 12 [19] 32 200
    // 200: 1 12 [19] 32 200
    // 12: 1 2 [12] 19 32
    // 1: 1 2 [3] 12 200
    // 2: 1 2 [3] 4 12
    // 3: 1 2 [3] 4 5
    // 4: not enough context
    // 5: not enough context

    auto filtered = Median(source, 5);

    CHECK_EQ(filtered(0), 1);
    CHECK_EQ(filtered(1), 5);
    CHECK_EQ(filtered(2), 5);
    CHECK_EQ(filtered(3), 9);
    CHECK_EQ(filtered(4), 12);
    CHECK_EQ(filtered(5), 12);
    CHECK_EQ(filtered(6), 19);
    CHECK_EQ(filtered(7), 19);
    CHECK_EQ(filtered(8), 19);
    CHECK_EQ(filtered(9), 12);
    CHECK_EQ(filtered(10), 3);
    CHECK_EQ(filtered(11), 3);
    CHECK_EQ(filtered(12), 3);
    CHECK_EQ(filtered(13), 4);
    CHECK_EQ(filtered(14), 5);
  }
}
}  // namespace common::math::filter
// NOLINTEND(*-magic-numbers)
