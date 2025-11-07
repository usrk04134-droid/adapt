
#include <doctest/doctest.h>

#include <chrono>
#include <memory>
#include <utility>

#include "scanner/joint_buffer/circular_joint_buffer.h"
#include "scanner/joint_buffer/joint_buffer.h"
#include "scanner/joint_model/joint_model.h"
#include "scanner/slice_provider/slice_provider_impl.h"

// NOLINTBEGIN(*-magic-numbers)
namespace scanner::slice_provider {

const auto abw0 = joint_model::Point{0.0, 0.0};
const auto abw1 = joint_model::Point{0.0, -0.008};
const auto abw2 = joint_model::Point{0.0, 0.0};
const auto abw3 = joint_model::Point{0.0, 0.0};
const auto abw4 = joint_model::Point{0.0, 0.0};
const auto abw5 = joint_model::Point{0.0, -0.008};
const auto abw6 = joint_model::Point{0.0, 0.0};

auto deep_joint = scanner::joint_model::ABWPoints{abw0, abw1, abw2, abw3, abw4, abw5, abw6};

TEST_SUITE("Slice provider") {
  TEST_CASE("Add") {
    auto joint_buffer          = std::make_unique<joint_buffer::CircularJointBuffer>();
    auto steady_clock_now_func = []() { return std::chrono::steady_clock::now(); };
    auto slice_provider = scanner::slice_provider::SliceProviderImpl(std::move(joint_buffer), steady_clock_now_func);

    scanner::joint_buffer::JointSlice slice = {
        .timestamp          = std::chrono::high_resolution_clock::now(),
        .profile            = {.points = deep_joint},
        .num_walls_found    = 2,
        .approximation_used = false,
    };
    // Only two walls in buffer and joint is deep
    slice_provider.AddSlice(slice);
    CHECK(!slice_provider.GetSlice());
      slice_provider.AddSlice(slice);
      slice_provider.AddSlice(slice);
      auto maybe_slice = slice_provider.GetSlice();
      CHECK(maybe_slice);

      auto tracking   = slice_provider.GetTrackingSlice();
      CHECK(tracking);
      auto confidence = std::get<2>(tracking.value());
      CHECK(confidence == slice_provider::SliceConfidence::HIGH);

    slice_provider.Reset();
    CHECK(!slice_provider.GetSlice());

    // One slice in buffer with two walls
    auto slice_mod = slice;
    slice_provider.AddSlice(slice_mod);
    slice_mod.num_walls_found = 1;
    slice_provider.AddSlice(slice_mod);
      slice_mod.num_walls_found = 0;
      slice_provider.AddSlice(slice_mod);

      maybe_slice = slice_provider.GetSlice();
      CHECK(maybe_slice);
      tracking    = slice_provider.GetTrackingSlice();
      CHECK(tracking);
      confidence  = std::get<2>(tracking.value());
      CHECK(confidence == slice_provider::SliceConfidence::MEDIUM);

    slice_provider.Reset();
    CHECK(!slice_provider.GetSlice());

    // Still quite high wall on one side
    auto slice_mod1            = slice;
    slice_mod1.num_walls_found = 1;
    slice_provider.AddSlice(slice_mod1);
    slice_mod1.num_walls_found = 1;
    slice_provider.AddSlice(slice_mod1);
    slice_mod1.num_walls_found = 1;
      slice_provider.AddSlice(slice_mod1);

      maybe_slice = slice_provider.GetSlice();
      CHECK(maybe_slice);
      tracking    = slice_provider.GetTrackingSlice();
      CHECK(tracking);
      confidence  = std::get<2>(tracking.value());
      CHECK(confidence == slice_provider::SliceConfidence::MEDIUM);

    slice_provider.Reset();
    CHECK(!slice_provider.GetSlice());

    // Still quite high wall on one side
    auto slice_mod2                = slice;
    slice_mod2.profile.points[1].y = 0.003;
    slice_mod2.profile.points[1].y = 0.0061;
    slice_mod2.num_walls_found     = 1;
    slice_provider.AddSlice(slice_mod2);
    slice_mod2.num_walls_found = 1;
    slice_provider.AddSlice(slice_mod2);
    slice_mod2.num_walls_found = 1;
      slice_provider.AddSlice(slice_mod2);

      maybe_slice = slice_provider.GetSlice();
      CHECK(maybe_slice);
      tracking    = slice_provider.GetTrackingSlice();
      CHECK(tracking);
      confidence  = std::get<2>(tracking.value());
      CHECK(confidence == slice_provider::SliceConfidence::MEDIUM);

    slice_provider.Reset();
    CHECK(!slice_provider.GetSlice());

    // Still quite high wall on one side.
    // But average number of walls is less than 1
    slice_mod2.num_walls_found = 1;
    slice_provider.AddSlice(slice_mod2);
    slice_mod2.num_walls_found = 0;
    slice_provider.AddSlice(slice_mod2);
    slice_mod2.num_walls_found = 0;
      slice_provider.AddSlice(slice_mod2);

      maybe_slice = slice_provider.GetSlice();
      CHECK(maybe_slice);
      tracking    = slice_provider.GetTrackingSlice();
      CHECK(tracking);
      confidence  = std::get<2>(tracking.value());
      CHECK(confidence == slice_provider::SliceConfidence::LOW);
  }
}
}  // namespace scanner::slice_provider
// NOLINTEND(*-magic-numbers)
