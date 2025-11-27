
#include <doctest/doctest.h>

#include <memory>
#include <string>
#include <trompeloeil.hpp>  // IWYU pragma: keep

#include "../calib_fix_joint_geometry_converter.h"
#include "../config_manager.h"
#include "common/file/yaml.h"
#include "main/joint_geometry/joint_geometry.h"
#include "mock/mock_factory.h"
#include "mock/mock_file_handler.h"

// NOLINTBEGIN(*-magic-numbers, misc-include-cleaner)

namespace configuration {

TEST_SUITE("Calibration Fixture Joint Geometry Converter") {
  TEST_CASE("Convert - read") {
    std::string yaml = R"(
    calibration_fixture_joint_geometry:
      # Upper joint width in mm
      upper_joint_width: 50.0
      # Groove depth in mm
      groove_depth: 28.0
      # Left joint angle in radians
      left_joint_angle: 0.1396
      # Right joint angle in radians
      right_joint_angle: 0.1396
      # Left max surface angle in radians
      left_max_surface_angle: 0.34906585
      # Right max surface angle in radians
      right_max_surface_angle: 0.34906585 
)";

    auto maybe_configuration_yaml = common::file::Yaml::FromString(yaml, TAG_CONF);

    CHECK_EQ(maybe_configuration_yaml.has_error(), false);

    auto map = maybe_configuration_yaml.value()->AsUnorderedMap();

    joint_geometry::JointGeometry ref = {.upper_joint_width_mm        = 50.0,
                                         .groove_depth_mm             = 28.0,
                                         .left_joint_angle_rad        = 0.1396,
                                         .right_joint_angle_rad       = 0.1396,
                                         .left_max_surface_angle_rad  = 0.34906585,
                                         .right_max_surface_angle_rad = 0.34906585,
                                         .welding_type                = joint_geometry::WeldingType::LONGITUDINAL};
    joint_geometry::JointGeometry data{};
    CHECK_EQ(CalibFixJointGeometryConverter::ToStruct(map, "/adaptio/config/configuration.yaml", data),
             boost::outcome_v2::success());
    // Expect configuration
    CHECK_EQ(ref.upper_joint_width_mm, data.upper_joint_width_mm);
    CHECK_EQ(ref.groove_depth_mm, data.groove_depth_mm);
    CHECK_EQ(ref.left_joint_angle_rad, data.left_joint_angle_rad);
    CHECK_EQ(ref.right_joint_angle_rad, data.right_joint_angle_rad);
    CHECK_EQ(ref.left_max_surface_angle_rad, data.left_max_surface_angle_rad);
    CHECK_EQ(ref.right_max_surface_angle_rad, data.right_max_surface_angle_rad);
  }
  TEST_CASE("Convert - exception") {
    std::string yaml = R"(
    calibration_fixture_joint_geometry:
      upper_joint_width: -nan
      groove_depth: 67.0
      left_joint_angle: 0.5236
      right_joint_angle: 0.5236
      left_max_surface_angle: 0.3491
      right_max_surface_angle: 0.3491)";

    auto maybe_configuration_yaml = common::file::Yaml::FromString(yaml, TAG_CONF);

    CHECK_EQ(maybe_configuration_yaml.has_error(), false);

    auto map = maybe_configuration_yaml.value()->AsUnorderedMap();

    joint_geometry::JointGeometry data{};
    CHECK_NE(CalibFixJointGeometryConverter::ToStruct(map, "/adaptio/config/configuration.yaml", data),
             boost::outcome_v2::success());
  }
}
// NOLINTEND(*-magic-numbers, misc-include-cleaner)
}  // namespace configuration
