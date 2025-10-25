#include "core/image/tilted_perspective_camera.h"

#include <boost/outcome/result.hpp>
#include <cmath>
#include <cstdint>
#include <Eigen/Core>
#include <memory>
#include <string>
#include <unordered_map>

#include "core/data/data_value.h"
#include "core/image/camera_model.h"
#include "core/scanner/scanner_calibration_configuration.h"

using core::image::TiltedPerspectiveCamera;
using core::image::TiltedPerspectiveCameraProperties;

using core::image::PlaneCoordinates;
using core::image::WorkspaceCoordinates;
using Eigen::Dynamic;
using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::RowMajor;
using Eigen::RowVectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;

TiltedPerspectiveCamera::TiltedPerspectiveCamera(const TiltedPerspectiveCameraProperties& camera_properties) {
  SetCameraProperties(camera_properties);
  if (!base_set_) {
    base_offset_x_ = camera_properties.config_fov.offset_x;
    base_width_    = camera_properties.config_fov.width;
    base_set_      = true;
  }
}

auto TiltedPerspectiveCamera::ImageToWorkspace(const PlaneCoordinates& image_coordinates,
                                               int vertical_crop_offset) const
    -> boost::outcome_v2::result<WorkspaceCoordinates> {
  using Eigen::all;

  auto intrinsic = camera_properties_.config_calib.intrinsic;
  auto extrinsic = camera_properties_.config_calib.extrinsic;
  auto fov       = camera_properties_.config_fov;

  Vector2d principal_point;
  principal_point << intrinsic.principal_point.x, intrinsic.principal_point.y;

  Vector2d pixel_pitch;
  pixel_pitch << intrinsic.pixel_pitch.x, intrinsic.pixel_pitch.y;

  Vector2d offset;
  offset << static_cast<double>(fov.offset_x), static_cast<double>(fov.offset_y + vertical_crop_offset);

  PlaneCoordinates coordinates = image_coordinates;

  // Offset the image coordinates to account for FOV
  coordinates = coordinates.colwise() + offset;

  // Scale points
  coordinates = coordinates / intrinsic.scaling_factors.w;

  // Scale the discrete image coordinates (pixels) into real world units.
  coordinates = CameraModel::ScaleFromPixels(coordinates, principal_point, pixel_pitch);

  // Account for the tilted optics
  WorkspaceCoordinates workspace_coordinates(3, coordinates.cols());
  workspace_coordinates << coordinates.array(), RowVectorXd::Ones(coordinates.cols()) / intrinsic.scaling_factors.w;
  workspace_coordinates = (intrinsic.scaling_factors.w * Hp_.inverse()) * workspace_coordinates;

  for (auto col : workspace_coordinates.colwise()) {
    col(0) = 1.0 / (intrinsic.scaling_factors.w * col(2)) * col(0);
    col(1) = 1.0 / (intrinsic.scaling_factors.w * col(2)) * col(1);
  }

  // Undistort the points
  PlaneCoordinates camera_coordinates = PlaneCoordinates::Zero(2, workspace_coordinates.cols());
  camera_coordinates << workspace_coordinates.row(0).array(), workspace_coordinates.row(1).array();
  auto K1            = intrinsic.K1 * intrinsic.scaling_factors.K1;  // NOLINT
  auto K2            = intrinsic.K2 * intrinsic.scaling_factors.K2;  // NOLINT
  auto K3            = intrinsic.K3 * intrinsic.scaling_factors.K3;  // NOLINT
  auto P1            = intrinsic.P1 * intrinsic.scaling_factors.P1;  // NOLINT
  auto P2            = intrinsic.P2 * intrinsic.scaling_factors.P2;  // NOLINT
  camera_coordinates = CameraModel::Undistort(camera_coordinates, K1, K2, K3, P1, P2);

  // Translate everything to the laser plane
  WorkspaceCoordinates wcs_coordinates = WorkspaceCoordinates::Zero(3, camera_coordinates.cols());
  wcs_coordinates = CameraModel::ImagePlaneToLaserPlane(camera_coordinates, extrinsic.rotation, extrinsic.translation,
                                                        intrinsic.focus_distance, intrinsic.scaling_factors.m,
                                                        intrinsic.scaling_factors.w);

  // Flip the Y coordinates
  wcs_coordinates(1, all) = -wcs_coordinates(1, all);

  return wcs_coordinates;
}

auto TiltedPerspectiveCamera::WorkspaceToImage(const WorkspaceCoordinates& workspace_coordinates,
                                               int vertical_crop_offset) const
    -> boost::outcome_v2::result<PlaneCoordinates> {
  using Eigen::all;
  using Eigen::Index;
  using Eigen::seq;
  using Eigen::VectorXd;

  auto intrinsic = camera_properties_.config_calib.intrinsic;
  auto extrinsic = camera_properties_.config_calib.extrinsic;
  auto fov       = camera_properties_.config_fov;

  Vector2d principal_point;
  principal_point << intrinsic.principal_point.x, intrinsic.principal_point.y;

  Vector2d pixel_pitch;
  pixel_pitch << intrinsic.pixel_pitch.x, intrinsic.pixel_pitch.y;

  Vector2d offset;
  offset << static_cast<double>(fov.offset_x), static_cast<double>(fov.offset_y + vertical_crop_offset);

  // Flip the Y coordinates
  WorkspaceCoordinates wcs_coordinates = workspace_coordinates;
  wcs_coordinates(1, all)              = -wcs_coordinates(1, all);

  PlaneCoordinates image_plane_coordinates = PlaneCoordinates::Zero(2, workspace_coordinates.cols());

  // Transform
  image_plane_coordinates = LaserPlaneToImagePlane(wcs_coordinates, extrinsic.rotation, extrinsic.translation,
                                                   intrinsic.focus_distance, intrinsic.scaling_factors.m);

  // Distort
  auto K1                 = intrinsic.K1 * intrinsic.scaling_factors.K1;  // NOLINT
  auto K2                 = intrinsic.K2 * intrinsic.scaling_factors.K2;  // NOLINT
  auto K3                 = intrinsic.K3 * intrinsic.scaling_factors.K3;  // NOLINT
  auto P1                 = intrinsic.P1 * intrinsic.scaling_factors.P1;  // NOLINT
  auto P2                 = intrinsic.P2 * intrinsic.scaling_factors.P2;  // NOLINT
  image_plane_coordinates = CameraModel::Distort(image_plane_coordinates, K1, K2, K3, P1, P2);

  // Untilt
  {
    WorkspaceCoordinates wcs_temp(3, image_plane_coordinates.cols());
    wcs_temp << image_plane_coordinates.array(),
        RowVectorXd::Ones(image_plane_coordinates.cols()) / intrinsic.scaling_factors.w;
    wcs_temp = intrinsic.scaling_factors.w * Hp_ * wcs_temp;

    for (auto col : wcs_temp.colwise()) {
      col(0) = 1.0 / (intrinsic.scaling_factors.w * col(2)) * col(0);
      col(1) = 1.0 / (intrinsic.scaling_factors.w * col(2)) * col(1);
    }

    image_plane_coordinates(seq(0, 1), all) = wcs_temp(seq(0, 1), all);
  }

  // Unscale to sensor
  image_plane_coordinates = CameraModel::ScaleToPixels(image_plane_coordinates, principal_point, pixel_pitch);

  // Remove offset
  for (auto col : image_plane_coordinates.colwise()) {
    col(0) = intrinsic.scaling_factors.w * col(0) - offset(0);
    col(1) = intrinsic.scaling_factors.w * col(1) - offset(1);
  }

  return image_plane_coordinates;
}

void TiltedPerspectiveCamera::SetCameraProperties(const TiltedPerspectiveCameraProperties& camera_properties) {
  auto intrinsic = camera_properties_.config_calib.intrinsic;

  if (camera_properties.config_calib.intrinsic.rho != intrinsic.rho ||
      camera_properties.config_calib.intrinsic.tau != intrinsic.tau ||
      camera_properties.config_calib.intrinsic.d != intrinsic.d ||
      camera_properties.config_calib.intrinsic.scaling_factors.m != intrinsic.scaling_factors.m) {
    Hp_ = CalculateTiltTransformationMatrix(
        camera_properties.config_calib.intrinsic.rho, camera_properties.config_calib.intrinsic.tau,
        camera_properties.config_calib.intrinsic.d * camera_properties.config_calib.intrinsic.scaling_factors.m);
  }

  camera_properties_ = camera_properties;
  if (!base_set_) {
    base_offset_x_ = camera_properties_.config_fov.offset_x;
    base_width_    = camera_properties_.config_fov.width;
    base_set_      = true;
  }
}

auto TiltedPerspectiveCamera::GetTiltTransformationMatrix() -> Eigen::Matrix<double, 3, 3, Eigen::RowMajor> {
  return Hp_;
}

auto TiltedPerspectiveCamera::CalculateTiltTransformationMatrix(double rho, double tau, double d)
    -> Eigen::Matrix<double, 3, 3, Eigen::RowMajor> {
  using Matrix = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

  // This math is straight from https://link.springer.com/content/pdf/10.1007/s11263-016-0964-8.pdf formula 25.
  // If we need to optimize this a reduced version is available in formula 32.

  // These are excluded from linting since we want to preserve the naming from the article.
  // NOLINTBEGIN(*-identifier-naming)
  // NOLINTBEGIN(*-identifier-length)
  Matrix Hp = Matrix::Zero();
  Matrix Ku = Matrix::Zero();
  Matrix Ks = Matrix::Zero();
  Matrix T  = Matrix::Zero();
  Matrix Rt = Matrix::Zero();
  Matrix R  = Matrix::Zero();
  // NOLINTEND(*-identifier-length)
  // NOLINTEND(*-identifier-naming)

  Rt(0, 0) = pow(cos(rho), 2) * (1 - cos(tau)) + cos(tau);
  Rt(0, 1) = cos(rho) * sin(rho) * (1 - cos(tau));
  Rt(0, 2) = sin(rho) * sin(tau);

  Rt(1, 0) = cos(rho) * sin(rho) * (1 - cos(tau));
  Rt(1, 1) = pow(sin(rho), 2) * (1 - cos(tau)) + cos(tau);
  Rt(1, 2) = -cos(rho) * sin(tau);

  Rt(2, 0) = -sin(rho) * sin(tau);
  Rt(2, 1) = cos(rho) * sin(tau);
  Rt(2, 2) = cos(tau);

  R = Rt.transpose();

  Ku(0, 0) = d;
  Ku(1, 1) = d;
  Ku(2, 2) = 1;

  Ks(0, 0) = d * Rt(2, 2);
  Ks(1, 1) = d * Rt(2, 2);
  Ks(2, 2) = 1;

  T(0, 0) = 1;
  T(0, 2) = -d * Rt(2, 0);
  T(1, 1) = 1;
  T(1, 2) = -d * Rt(2, 1);
  T(2, 2) = 1;

  Hp = T * Ks * R * Ku.inverse();

  return Hp;
}

auto TiltedPerspectiveCameraProperties::FromUnorderedMap(
    const std::unordered_map<std::string, core::data::DataValue>& map) -> TiltedPerspectiveCameraProperties {
  TiltedPerspectiveCameraProperties camera_properties;

  // Try get serial number. Not all calibration files has it
  auto serial_int = map.at("camera/scanner_serial_number").Value<std::int64_t>();
  if (serial_int.has_value()) {
    camera_properties.config_calib.scanner_serial_number = std::to_string(serial_int.value());
  } else {
    auto serial = map.at("camera/scanner_serial_number").Value<std::string>();
    if (serial.has_value()) {
      camera_properties.config_calib.scanner_serial_number = serial.value();
    }
  }

  // Read the scaling parameters first so we can use them to scale the rest.
  camera_properties.config_calib.intrinsic.scaling_factors.w =
      map.at("camera/intrinsic/scaling_factors/w").Value<double>().value();
  camera_properties.config_calib.intrinsic.scaling_factors.m =
      map.at("camera/intrinsic/scaling_factors/m").Value<double>().value();
  camera_properties.config_calib.intrinsic.scaling_factors.K1 =
      map.at("camera/intrinsic/scaling_factors/K1").Value<double>().value();
  camera_properties.config_calib.intrinsic.scaling_factors.K2 =
      map.at("camera/intrinsic/scaling_factors/K2").Value<double>().value();
  camera_properties.config_calib.intrinsic.scaling_factors.K3 =
      map.at("camera/intrinsic/scaling_factors/K3").Value<double>().value();
  camera_properties.config_calib.intrinsic.scaling_factors.P1 =
      map.at("camera/intrinsic/scaling_factors/P1").Value<double>().value();
  camera_properties.config_calib.intrinsic.scaling_factors.P2 =
      map.at("camera/intrinsic/scaling_factors/P2").Value<double>().value();

  camera_properties.config_calib.intrinsic.projection_center_distance =
      map.at("camera/intrinsic/projection_center_distance").Value<double>().value();
  camera_properties.config_calib.intrinsic.focus_distance =
      map.at("camera/intrinsic/focus_distance").Value<double>().value();
  camera_properties.config_calib.intrinsic.principal_point.x =
      map.at("camera/intrinsic/principal_point/x").Value<double>().value();
  camera_properties.config_calib.intrinsic.principal_point.y =
      map.at("camera/intrinsic/principal_point/y").Value<double>().value();
  camera_properties.config_calib.intrinsic.pixel_pitch.x =
      map.at("camera/intrinsic/pixel_pitch/x").Value<double>().value();
  camera_properties.config_calib.intrinsic.pixel_pitch.y =
      map.at("camera/intrinsic/pixel_pitch/y").Value<double>().value();
  camera_properties.config_calib.intrinsic.rho = map.at("camera/intrinsic/rho").Value<double>().value();
  camera_properties.config_calib.intrinsic.tau = map.at("camera/intrinsic/tau").Value<double>().value();
  camera_properties.config_calib.intrinsic.d   = map.at("camera/intrinsic/d").Value<double>().value();
  camera_properties.config_calib.intrinsic.K1  = map.at("camera/intrinsic/K1").Value<double>().value();
  camera_properties.config_calib.intrinsic.K2  = map.at("camera/intrinsic/K2").Value<double>().value();
  camera_properties.config_calib.intrinsic.K3  = map.at("camera/intrinsic/K3").Value<double>().value();
  camera_properties.config_calib.intrinsic.P1  = map.at("camera/intrinsic/P1").Value<double>().value();
  camera_properties.config_calib.intrinsic.P2  = map.at("camera/intrinsic/P2").Value<double>().value();

  auto r_data = map.at("camera/extrinsic/R").Value<core::data::Matrix>().value();
  for (int i = 0; i < r_data.rows; i++) {
    for (int j = 0; j < r_data.columns; j++) {
      camera_properties.config_calib.extrinsic.rotation(i, j) = r_data.data.get()[i * r_data.columns + j];
    }
  }

  auto t_data = map.at("camera/extrinsic/t").Value<core::data::Matrix>().value();
  for (int i = 0; i < t_data.rows; i++) {
    for (int j = 0; j < t_data.columns; j++) {
      camera_properties.config_calib.extrinsic.translation(i, j) = t_data.data.get()[i * t_data.columns + j];
    }
  }

  return camera_properties;
}

#ifndef DOCTEST_CONFIG_DISABLE
// NOLINTBEGIN(*-magic-numbers)

#include <doctest/doctest.h>

#include <cfloat>
#include <cstdlib>

#include "core/file/yaml.h"

using core::file::Yaml;

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

TEST_SUITE("Test tilted perspective camera") {
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

// NOLINTEND(*-magic-numbers)
#endif
