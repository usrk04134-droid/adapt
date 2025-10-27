#include "scanner/image/tilted_perspective_camera.h"

#include <boost/outcome/result.hpp>
#include <cmath>
#include <cstdint>
#include <Eigen/Core>
#include <memory>
#include <string>
#include <unordered_map>

#include "common/data/data_value.h"
#include "scanner/image/camera_model.h"
#include "scanner/scanner_calibration_configuration.h"

using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::RowVectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace scanner::image {

TiltedPerspectiveCamera::TiltedPerspectiveCamera(const TiltedPerspectiveCameraProperties& camera_properties) {
  SetCameraProperties(camera_properties);
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
  // Account for dynamic horizontal ROI (OffsetX) adjustments at runtime
  offset << static_cast<double>(fov.offset_x + dynamic_offset_x_pixels_),
      static_cast<double>(fov.offset_y + vertical_crop_offset);

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
  // Account for dynamic horizontal ROI (OffsetX) adjustments at runtime
  offset << static_cast<double>(fov.offset_x + dynamic_offset_x_pixels_),
      static_cast<double>(fov.offset_y + vertical_crop_offset);

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
    const std::unordered_map<std::string, common::data::DataValue>& map) -> TiltedPerspectiveCameraProperties {
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

  auto r_data = map.at("camera/extrinsic/R").Value<common::data::Matrix>().value();
  for (int i = 0; i < r_data.rows; i++) {
    for (int j = 0; j < r_data.columns; j++) {
      camera_properties.config_calib.extrinsic.rotation(i, j) = r_data.data.get()[i * r_data.columns + j];
    }
  }

  auto t_data = map.at("camera/extrinsic/t").Value<common::data::Matrix>().value();
  for (int i = 0; i < t_data.rows; i++) {
    for (int j = 0; j < t_data.columns; j++) {
      camera_properties.config_calib.extrinsic.translation(i, j) = t_data.data.get()[i * t_data.columns + j];
    }
  }

  return camera_properties;
}
}  // namespace scanner::image
