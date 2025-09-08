#include "common/file/yaml.h"

#include <doctest/doctest.h>

#include <memory>
#include <string>
#include <unordered_map>

#include "common/data/data_value.h"

// NOLINTBEGIN(*-magic-numbers)
namespace common::file {

TEST_SUITE("YAML parsing") {
  TEST_CASE("Empty document") {
    std::string yaml = "---";

    auto y          = Yaml::FromString(yaml, "adaptio").value();
    auto parameters = y->AsUnorderedMap();

    CHECK_EQ(parameters.size(), 0);
  }

  TEST_CASE("Simple document") {
    std::string yaml = R"(pi: 3.141500000000000e+00
file_path: /path)";

    auto y          = Yaml::FromString(yaml, "adaptio").value();
    auto parameters = y->AsUnorderedMap();

    CHECK_EQ(parameters.size(), 2);
    CHECK(parameters.find("adaptio/pi") != parameters.end());
    CHECK_LT(parameters.at("adaptio/pi").Value<double>().value() - 3.1415, 0.0001);
    CHECK_EQ(parameters.at("adaptio/file_path").Value<std::string>(), "/path");

    CHECK_EQ(yaml, y->ToString(parameters).value());
  }

  TEST_CASE("Nested document") {
    std::string yaml = R"(
---

constants:
  pi: 3.1415
)";

    std::string yaml_out = R"(constants:
  pi: 2.000000000000000e+00)";

    auto y          = Yaml::FromString(yaml, "adaptio").value();
    auto parameters = y->AsUnorderedMap();

    CHECK_EQ(parameters.size(), 1);
    CHECK(parameters.find("adaptio/constants/pi") != parameters.end());
    CHECK_LT(parameters.at("adaptio/constants/pi").Value<double>().value() - 3.1415, 0.0001);

    // Modify data
    parameters.at("adaptio/constants/pi") = common::data::DataValue(2.0);

    CHECK_EQ(yaml_out, y->ToString(parameters).value());
  }

  TEST_CASE("Sequence (array) document") {
    std::string yaml = R"(
---

R: !matrix
  columns: 3
  rows: 3
  data:
    [ 1.1, 2.0, 4.0,
      1.0, 2.0, 4.0,
      1.0, 2.0, 4.0 ]
)";

    std::string out_yaml = R"(R: !<!matrix>
  columns: 3
  rows: 3
  data: [1.100000000000000e+00, 2.000000000000000e+00, 4.000000000000000e+00, 1.000000000000000e+00, 3.000000000000000e+00, 4.000000000000000e+00, 1.000000000000000e+00, 2.000000000000000e+00, 4.000000000000000e+00])";
    auto y               = Yaml::FromString(yaml, "adaptio").value();
    auto parameters      = y->AsUnorderedMap();

    CHECK_EQ(parameters.size(), 1);
    CHECK(parameters.find("adaptio/R") != parameters.end());
    CHECK(parameters.at("adaptio/R").Value<common::data::Matrix>().has_value());

    // Modify matrix
    parameters.at("adaptio/R").Value<common::data::Matrix>()->data.get()[4] = 3.0;
    auto yaml_string                                                        = y->ToString(parameters).value();
    CHECK_EQ(yaml_string, out_yaml);

    // From/To once more
    y           = Yaml::FromString(yaml_string, "adaptio").value();
    yaml_string = y->ToString(y->AsUnorderedMap()).value();
    CHECK_EQ(yaml_string, out_yaml);
  }

  TEST_CASE("Example configuration") {
    std::string yaml = R"(
---

# The camera parameters are set during calibration
camera_parameters:
  # The intrinsic camera parameters are represented as a matrix:
  # [
  #   fx, 0,  cx,
  #    0, fy, cy,
  #    0,  0,  1,
  # ]
  #
  # Where:
  #   fx = focal length in x
  #   fy = focal length in y (fy = fx * a, a = aspect ratio)
  #   cx = optical center in x
  #   cy = optical center in y
  intrinsic: !matrix
    rows: 3
    columns: 3
    data: [ 1.0, 0.0, 0.5,
            0.0, 1.0, 0.5,
            0.0, 0.0, 1.0 ]
  fov:
    width: 3500
    height: 2500
    offset_x: 298
    offset_y: 0

  extrinsic:
    # Rotation matrix
    R: !matrix
      rows: 3
      columns: 3
      data: [ 9.999974673412257431e-01, 2.039705193809659024e-03,  9.512696023625968975e-04,
              0.000000000000000000e+00, 4.226691551490259768e-01, -9.062840533108859065e-01,
              -2.250624609754632317e-03, 9.062817580026263364e-01,  4.226680846722816187e-01 ]

    # Translation vector
    t: !matrix
      rows: 3
      columns: 1
      data: [ 0.000000000000000000e+00, 0.000000000000000000e+00, 4.087606157143235386e-01 ]

    # Distortion coefficients
    D: !matrix
      rows: 5
      columns: 1
      data: [ 0.1, 0.01, -0.001, 0, 0 ]

image_processing:
  # Initial guesses
  left_joint_angle: 0.1396263401595 # 8.0 / 360.0 * 2 * PI
  right_joint_angle: 0.1396263401595 # 8.0 / 360.0 * 2 * PI
  left_groove_depth: 0.0
  right_groove_depth: 0.0
)";

    auto y          = Yaml::FromString(yaml, "adaptio").value();
    auto parameters = y->AsUnorderedMap();

    CHECK(parameters.find("adaptio/camera_parameters/intrinsic") != parameters.end());
    CHECK(parameters.find("adaptio/camera_parameters/fov/width") != parameters.end());
    CHECK(parameters.find("adaptio/camera_parameters/fov/height") != parameters.end());
    CHECK(parameters.find("adaptio/camera_parameters/fov/offset_x") != parameters.end());
    CHECK(parameters.find("adaptio/camera_parameters/fov/offset_y") != parameters.end());
    CHECK(parameters.find("adaptio/camera_parameters/extrinsic/R") != parameters.end());
    CHECK(parameters.find("adaptio/camera_parameters/extrinsic/t") != parameters.end());
    CHECK(parameters.find("adaptio/camera_parameters/extrinsic/D") != parameters.end());
    CHECK(parameters.find("adaptio/image_processing/left_joint_angle") != parameters.end());
    CHECK(parameters.find("adaptio/image_processing/right_joint_angle") != parameters.end());
    CHECK(parameters.find("adaptio/image_processing/left_groove_depth") != parameters.end());
    CHECK(parameters.find("adaptio/image_processing/right_groove_depth") != parameters.end());

    // To/From
    auto yaml_string = y->ToString(parameters).value();

    y               = Yaml::FromString(yaml_string, "adaptio").value();
    auto new_params = y->AsUnorderedMap();

    CHECK(new_params.find("adaptio/camera_parameters/intrinsic") != parameters.end());
    CHECK(new_params.find("adaptio/camera_parameters/fov/width") != parameters.end());
    CHECK(new_params.find("adaptio/camera_parameters/fov/height") != parameters.end());
    CHECK(new_params.find("adaptio/camera_parameters/fov/offset_x") != parameters.end());
    CHECK(new_params.find("adaptio/camera_parameters/fov/offset_y") != parameters.end());
    CHECK(new_params.find("adaptio/camera_parameters/extrinsic/R") != parameters.end());
    CHECK(new_params.find("adaptio/camera_parameters/extrinsic/t") != parameters.end());
    CHECK(new_params.find("adaptio/camera_parameters/extrinsic/D") != parameters.end());
    CHECK(new_params.find("adaptio/image_processing/left_joint_angle") != parameters.end());
    CHECK(new_params.find("adaptio/image_processing/right_joint_angle") != parameters.end());
    CHECK(new_params.find("adaptio/image_processing/left_groove_depth") != parameters.end());
    CHECK(new_params.find("adaptio/image_processing/right_groove_depth") != parameters.end());
  }
}
}  // namespace common::file
// NOLINTEND(*-magic-numbers)
