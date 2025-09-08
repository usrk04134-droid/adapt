#include "../weld_axis_filter.h"

#include <doctest/doctest.h>

#include <cstddef>
#include <vector>

using controller::WeldAxisFilterMedianImpl;

// NOLINTBEGIN(*-magic-numbers, *-optional-access)

TEST_SUITE("WeldAxisFilter") {
  TEST_CASE("Initialization") {
    SUBCASE("Zero window size should default to 1") {
      WeldAxisFilterMedianImpl filter(0);
      CHECK(filter.GetWindowSize() == 1);
    }

    SUBCASE("Even window size should increment to make it odd") {
      WeldAxisFilterMedianImpl filter(4);
      CHECK(filter.GetWindowSize() == 5);
    }

    SUBCASE("Odd window size remains unchanged") {
      WeldAxisFilterMedianImpl filter(5);
      CHECK(filter.GetWindowSize() == 5);
    }
  }

  TEST_CASE("ProcessSignal") {
    WeldAxisFilterMedianImpl filter(5);
    std::vector<float> input_signal = {
        0.13304093480110168, 0.13304093480110168, 0.13304093480110168, 0.130153089761734,   0.128287672996521,
        0.12901651859283447, 0.12901651859283447, 0.12901651859283447, 0.12901651859283447, 0.12602569162845612,
        0.11424878984689713, 0.11424878984689713, 0.1230638325214386,  0.12494136393070221, 0.1220761239528656,
        0.11605194211006165, 0.11605194211006165, 0.12646663188934326, 0.12378576397895813, 0.12312660366296768,
        0.12112551182508469, 0.12306084483861923, 0.12306084483861923};

    std::vector<float> expected_medians = {
        0,        0,        0.133041, 0.133041, 0.133041, 0.130153, 0.129017, 0.129017,
        0.129017, 0.129017, 0.129017, 0.126026, 0.123064, 0.123064, 0.122076, 0.122076,
        0.122076, 0.122076, 0.122076, 0.123127, 0.123127, 0.123127, 0.123061,
    };

    for (size_t i = 0; i < input_signal.size(); i++) {
      float median = filter.ProcessSignal(input_signal[i]);
      CHECK(median == doctest::Approx(expected_medians[i]));
    }
  }

  TEST_CASE("ClearSignalBuffer") {
    WeldAxisFilterMedianImpl filter(5);
    filter.ProcessSignal(10);
    filter.ProcessSignal(20);
    filter.ClearSignalBuffer();

    float median = filter.ProcessSignal(0);
    CHECK(median == 0);
  }
}

// NOLINTEND(*-magic-numbers, *-optional-access)
