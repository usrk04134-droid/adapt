#include <doctest/doctest.h>

#include <memory>
#include <string>
#include <trompeloeil.hpp>  // IWYU pragma: keep

#include "../config_manager.h"
#include "common/tolerances/tolerances_configuration.h"
#include "configuration/tolerances_converter.h"
#include "mock/mock_factory.h"
#include "mock/mock_file_handler.h"

// NOLINTBEGIN(*-magic-numbers, misc-include-cleaner)

namespace configuration {
TEST_SUITE("Tolerances Converter") {
  TEST_CASE("Convert") {
    std::string yaml = R"(
    tolerances:
      joint_geometry:
        upper_width: 10.0
        surface_angle: 0.174532925
        wall_angle: 0.13962634 )";

    auto maybe_configuration_yaml = common::file::Yaml::FromString(yaml, TAG_CONF);

    CHECK_EQ(maybe_configuration_yaml.has_error(), false);

    auto map = maybe_configuration_yaml.value()->AsUnorderedMap();

    tolerances::Configuration ref{};

    ref.joint_geometry.upper_width   = 10.0;
    ref.joint_geometry.surface_angle = 0.174532925;
    ref.joint_geometry.wall_angle    = 0.13962634;

    tolerances::Configuration data{};
    CHECK_EQ(TolerancesConverter::ToStruct(map, "/adaptio/config/configuration.yaml", data),
             boost::outcome_v2::success());

    CHECK_EQ(ref.joint_geometry.surface_angle, data.joint_geometry.surface_angle);
    CHECK_EQ(ref.joint_geometry.wall_angle, data.joint_geometry.wall_angle);
    CHECK_EQ(ref.joint_geometry.upper_width, data.joint_geometry.upper_width);
  }

  TEST_CASE("Convert - Exception") {
    std::string yaml = R"(
    tolerances:
      joint_geometry:
        upper_width: 10
        surface_angled: 0.174532925
        wall_angle: 0.13962634 )";

    auto maybe_configuration_yaml = common::file::Yaml::FromString(yaml, TAG_CONF);

    CHECK_EQ(maybe_configuration_yaml.has_error(), false);

    auto map = maybe_configuration_yaml.value()->AsUnorderedMap();

    tolerances::Configuration data{};
    CHECK_NE(TolerancesConverter::ToStruct(map, "/adaptio/config/configuration.yaml", data),
             boost::outcome_v2::success());
  }
}
// NOLINTEND(*-magic-numbers, misc-include-cleaner)

}  // namespace configuration
