
#include "scanner/image/tilted_perspective_camera.h"

#include <doctest/doctest.h>

#include <boost/outcome/result.hpp>
#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <Eigen/Core>
#include <memory>
#include <string>

#include "common/file/yaml.h"
#include "scanner/image/camera_model.h"

using common::file::Yaml;

// NOLINTBEGIN(*-magic-numbers)
namespace scanner::image {

auto yaml = R"#(
---

# The camera parameters are set during calibration
scanner_serial_number: 1234
intrinsic:
  projection_center_distance: 0.0
  focus_distance: 3.744565963745117188e+00
  principal_point:
    x: 9.869480729103088379e-01
    y: 7.230033874511718750e-01
  pixel_pitch:
    x: 2.74e-06
    y: 2.74e-06

  rho: 3.141447305679321289e+00
  tau: 1.645262539386749268e-01
  d: 6.015305519104003906e-01
  K1: 3.780014812946319580e-03
  K2: -1.993117621168494225e-03
  K3: 5.228068857832113281e-07
  P1: -1.876385213108733296e-04
  P2: -5.847600405104458332e-04

  scaling_factors:
    w: 0.007093
    m: 0.1
    K1: 0.1
    K2: 0.1
    K3: 0.1
    P1: 0.1
    P2: 0.1


extrinsic:
  # Rotation matrix
  R: !matrix
    rows: 3
    columns: 3
    data: [ 9.999974673412257431e-01, 2.039705193809659024e-03, 9.512696023625968975e-04,
            0.000000000000000000e+00, 4.226691551490259768e-01, -9.062840533108859065e-01,
            -2.250624609754632317e-03, 9.062817580026263364e-01, 4.226680846722816187e-01]

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
)#";

TEST_SUITE("TiltedPerspectiveCamera") {
  TEST_CASE("Calculate tilt transformation matrix") {
    TiltedPerspectiveCameraProperties properties;

    properties.config_calib.intrinsic.rho               = 3.141447305679321289e+00;
    properties.config_calib.intrinsic.tau               = 1.645262539386749268e-01;
    properties.config_calib.intrinsic.scaling_factors.m = 0.1;
    properties.config_calib.intrinsic.d                 = 5.489320311280183606e-01;
    properties.config_fov.width                         = 3500;
    properties.config_fov.height                        = 2500;
    properties.config_fov.offset_x                      = 298;
    properties.config_fov.offset_y                      = 0;

    TiltedPerspectiveCamera camera(properties);

    auto Hp = camera.GetTiltTransformationMatrix();

    // The target values are calculated with the Julia scripts in tests/math.
    CHECK_LE(abs(abs(Hp(0, 0)) - 0.986496058830029), DBL_EPSILON);
    CHECK_LE(abs(abs(Hp(0, 1)) - 1.96276964601327e-6), DBL_EPSILON);
    CHECK_LE(abs(abs(Hp(0, 2)) - 0.0), DBL_EPSILON);
    CHECK_LE(abs(abs(Hp(1, 0)) - 1.96276964601327e-6), DBL_EPSILON);
    CHECK_LE(abs(abs(Hp(1, 1)) - 0.999999999714716), 3 * DBL_EPSILON);
    CHECK_LE(abs(abs(Hp(1, 2)) - 0.0), DBL_EPSILON);
    CHECK_LE(abs(abs(Hp(2, 0)) - 0.000433674957811098), DBL_EPSILON);
    CHECK_LE(abs(abs(Hp(2, 1)) - 2.98370271267747), 4 * DBL_EPSILON);
    CHECK_LE(abs(abs(Hp(2, 2)) - 0.986496058544744), 2 * DBL_EPSILON);
  }

  TEST_CASE("Tilt transformation") {
    TiltedPerspectiveCameraProperties properties;
    properties.config_calib.intrinsic.rho               = 3.141447305679321289e+00;
    properties.config_calib.intrinsic.tau               = 1.645262539386749268e-01;
    properties.config_calib.intrinsic.scaling_factors.m = 0.1;
    properties.config_calib.intrinsic.d                 = 5.489320311280183606e-01;
    properties.config_fov.width                         = 3500;
    properties.config_fov.height                        = 2500;
    properties.config_fov.offset_x                      = 298;
    properties.config_fov.offset_y                      = 0;

    TiltedPerspectiveCamera camera(properties);

    auto Hp = camera.GetTiltTransformationMatrix();

    WorkspaceCoordinates A(3, 5);
    A << 1, 2, 3, 4, 5, 3, 1, 4, 1, 7, 7, 2, 9, 4, 9;

    auto T = Hp * A;

    CHECK_LE(abs(T(0, 0) - 0.986501947138967), 10e-13);
    CHECK_LE(abs(T(0, 1) - 1.97299408042970), 10e-13);
    CHECK_LE(abs(T(0, 2) - 2.95949602756867), 10e-13);
    CHECK_LE(abs(T(0, 3) - 3.94598619808976), 10e-13);
    CHECK_LE(abs(T(0, 4) - 4.93249403353767), 10e-13);
    CHECK_LE(abs(T(1, 0) - 3.00000196191379), 10e-13);
    CHECK_LE(abs(T(1, 1) - 1.00000392525401), 10e-13);
    CHECK_LE(abs(T(1, 2) - 4.00000588716780), 10e-13);
    CHECK_LE(abs(T(1, 3) - 1.00000785079330), 10e-13);
    CHECK_LE(abs(T(1, 4) - 7.00000981185124), 10e-13);
    CHECK_LE(abs(T(2, 0) - 15.8570142228034), 10e-13);
    CHECK_LE(abs(T(2, 1) - 4.95756217968258), 10e-13);
    CHECK_LE(abs(T(2, 2) - 20.8145764024860), 10e-13);
    CHECK_LE(abs(T(2, 3) - 6.93142164668769), 10e-13);
    CHECK_LE(abs(T(2, 4) - 29.7665518904340), 10e-13);
  }

  TEST_CASE("Image to workspace") {
    using Eigen::Index;

    PlaneCoordinates image(2, 10);
    image << 1, 2, 3, 4, 5, 3546, 3547, 3548, 3549, 3500, 503.5, 503.148, 503, 502.572, 502.604, 521, 521.043, 521.026,
        520.94, 520.854;

    auto parsed_yaml = Yaml::FromString(yaml, "camera");

    if (parsed_yaml.has_error()) {
      CHECK(false);
    }

    auto properties = TiltedPerspectiveCameraProperties::FromUnorderedMap(parsed_yaml.value()->AsUnorderedMap());
    properties.config_fov.width    = 3500;
    properties.config_fov.height   = 2500;
    properties.config_fov.offset_x = 298;
    properties.config_fov.offset_y = 0;

    auto camera = TiltedPerspectiveCamera(properties);

    auto maybe_wcs = camera.ImageToWorkspace(image, 0);

    if (maybe_wcs.has_error()) {
      CHECK(false);
    }

    auto wcs = maybe_wcs.value();

    // The target values are calculated by inputting the points into the original ICS2WCS python function
    CHECK_LE(abs(wcs(0, 0) - -0.07252012), 1.0e-5);
    CHECK_LE(abs(wcs(0, 1) - -0.07248341), 1.0e-5);
    CHECK_LE(abs(wcs(0, 2) - -0.07244929), 1.0e-5);
    CHECK_LE(abs(wcs(0, 3) - -0.07241163), 1.0e-5);
    CHECK_LE(abs(wcs(0, 4) - -0.07237979), 1.0e-5);
    CHECK_LE(abs(wcs(0, 5) - 0.04186408), 1.0e-5);
    CHECK_LE(abs(wcs(0, 6) - 0.04189669), 1.0e-5);
    CHECK_LE(abs(wcs(0, 7) - 0.04192888), 1.0e-5);
    CHECK_LE(abs(wcs(0, 8) - 0.04196058), 1.0e-5);
    CHECK_LE(abs(wcs(0, 9) - 0.0403769), 1.0e-5);

    CHECK_LE(abs(wcs(1, 0) - 0.10295671), 1.0e-5);
    CHECK_LE(abs(wcs(1, 1) - 0.10297692), 1.0e-5);
    CHECK_LE(abs(wcs(1, 2) - 0.10298541), 1.0e-5);
    CHECK_LE(abs(wcs(1, 3) - 0.10300998), 1.0e-5);
    CHECK_LE(abs(wcs(1, 4) - 0.10300812), 1.0e-5);
    CHECK_LE(abs(wcs(1, 5) - 0.10187241), 1.0e-5);
    CHECK_LE(abs(wcs(1, 6) - 0.10186991), 1.0e-5);
    CHECK_LE(abs(wcs(1, 7) - 0.10187089), 1.0e-5);
    CHECK_LE(abs(wcs(1, 8) - 0.10187585), 1.0e-5);
    CHECK_LE(abs(wcs(1, 9) - 0.10188133), 1.0e-5);
  }

  TEST_CASE("Image to workspace to image") {
    using Eigen::Index;

    PlaneCoordinates image(2, 10);
    image << 1, 2, 3, 4, 5, 3546, 3547, 3548, 3549, 3500, 503.5, 503.148, 503, 502.572, 502.604, 521, 521.043, 521.026,
        520.94, 520.854;

    auto parsed_yaml = Yaml::FromString(yaml, "camera");

    if (parsed_yaml.has_error()) {
      CHECK(false);
    }

    auto properties = TiltedPerspectiveCameraProperties::FromUnorderedMap(parsed_yaml.value()->AsUnorderedMap());
    properties.config_fov.width    = 3500;
    properties.config_fov.height   = 2500;
    properties.config_fov.offset_x = 298;
    properties.config_fov.offset_y = 0;

    auto camera = TiltedPerspectiveCamera(properties);

    auto maybe_wcs = camera.ImageToWorkspace(image, 0);

    if (maybe_wcs.has_error()) {
      CHECK(false);
    }

    auto wcs = maybe_wcs.value();

    auto maybe_img = camera.WorkspaceToImage(wcs, 0);

    if (maybe_img.has_error()) {
      CHECK(false);
    }

    auto img = maybe_img.value();

    // Accept a deviance of < 0.1 pixels
    CHECK_LE(fabs(img(0, 0) - 1.0), 1.0e-1);
    CHECK_LE(fabs(img(0, 1) - 2.0), 1.0e-1);
    CHECK_LE(fabs(img(0, 2) - 3.0), 1.0e-1);
    CHECK_LE(fabs(img(0, 3) - 4.0), 1.0e-1);
    CHECK_LE(fabs(img(0, 4) - 5.0), 1.0e-1);
    CHECK_LE(fabs(img(0, 5) - 3546.0), 1.0e-1);
    CHECK_LE(fabs(img(0, 6) - 3547.0), 1.0e-1);
    CHECK_LE(fabs(img(0, 7) - 3548.0), 1.0e-1);
    CHECK_LE(fabs(img(0, 8) - 3549.0), 1.0e-1);
    CHECK_LE(fabs(img(0, 9) - 3500.0), 1.0e-1);

    CHECK_LE(fabs(img(1, 0) - 503.5), 1.0e-1);
    CHECK_LE(fabs(img(1, 1) - 503.148), 1.0e-1);
    CHECK_LE(fabs(img(1, 2) - 503), 1.0e-1);
    CHECK_LE(fabs(img(1, 3) - 502.572), 1.0e-1);
    CHECK_LE(fabs(img(1, 4) - 502.604), 1.0e-1);
    CHECK_LE(fabs(img(1, 5) - 521), 1.0e-1);
    CHECK_LE(fabs(img(1, 6) - 521.043), 1.0e-1);
    CHECK_LE(fabs(img(1, 7) - 521.026), 1.0e-1);
    CHECK_LE(fabs(img(1, 8) - 520.94), 1.0e-1);
    CHECK_LE(fabs(img(1, 9) - 520.854), 1.0e-1);
  }
}

}  // namespace scanner::image
// NOLINTEND(*-magic-numbers)
