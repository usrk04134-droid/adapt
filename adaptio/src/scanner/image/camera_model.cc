#include "scanner/image/camera_model.h"

#include <ceres/ceres.h>  // IWYU pragma: keep
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <ceres/types.h>

#include <Eigen/Core>
#include <string>
#include <system_error>

using Eigen::RowVectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace scanner::image {

auto CameraModel::ScaleFromPixels(const PlaneCoordinates& pixel_coordinates, const Vector2d& principal_point,
                                  const Vector2d& pixel_pitch) -> PlaneCoordinates {
  PlaneCoordinates coordinates(2, pixel_coordinates.cols());
  coordinates << pixel_coordinates.array();

  coordinates.row(0) = pixel_pitch(0) * coordinates.row(0).array() - principal_point(0);
  coordinates.row(1) = pixel_pitch(1) * coordinates.row(1).array() - principal_point(1);

  return coordinates;
}

auto CameraModel::ScaleToPixels(const PlaneCoordinates& length_coordinates, const Vector2d& principal_point,
                                const Vector2d& pixel_pitch) -> PlaneCoordinates {
  PlaneCoordinates coordinates(2, length_coordinates.cols());
  coordinates << length_coordinates.array();

  coordinates.row(0) = coordinates.row(0).array() / pixel_pitch(0) + principal_point(0) / pixel_pitch(0);
  coordinates.row(1) = coordinates.row(1).array() / pixel_pitch(1) + principal_point(1) / pixel_pitch(1);

  return coordinates;
}

auto CameraModel::ImagePlaneToLaserPlane(const PlaneCoordinates& image_coordinates, const RotationMatrix& rotation,
                                         const TranslationVector& translation, double focus_distance, double m,
                                         double w) -> WorkspaceCoordinates {
  using Coordinates = Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor>;

  // Here we need a z-component as we go from 2D (image) to 3D (workspace)
  auto z_component = RowVectorXd::Zero(image_coordinates.cols()).array() + w * focus_distance;
  Coordinates coordinates(image_coordinates.rows() + z_component.rows(), z_component.cols());
  coordinates << (w * image_coordinates.array()), z_component;

  // Transform "backwards" to WCS coordinates
  // pi = R * pw + t -> R^-1 * (pi - t)
  coordinates = rotation.transpose() * coordinates;

  // The projection center is at (0,0) in the image plane, so in wcs this will be R^-1 * ([0,0,0] - t) = -R*t
  const Vector3d projection_center = -1.0 * m * rotation.transpose() * (translation / m);

  // The directional vectors of the optical rays that passes through the projection center and hits the points in the
  // image plane.
  // const Coordinates optical_rays = coordinates.colwise() - projection_center;

  // All points in this plane are at Z = 0 per definition.
  WorkspaceCoordinates wcs_coordinates = WorkspaceCoordinates::Zero(3, coordinates.cols());

  // This projects the optical rays onto the laser plane.
  wcs_coordinates.row(0) =
      -projection_center(2) / coordinates.row(2).array() * coordinates.row(0).array() + projection_center(0);
  wcs_coordinates.row(1) =
      -projection_center(2) / coordinates.row(2).array() * coordinates.row(1).array() + projection_center(1);

  return wcs_coordinates;
}

auto CameraModel::LaserPlaneToImagePlane(const WorkspaceCoordinates& laser_coordinates, const RotationMatrix& rotation,
                                         const TranslationVector& translation, double focus_distance, double m)
    -> PlaneCoordinates {
  PlaneCoordinates coordinates         = PlaneCoordinates::Zero(2, laser_coordinates.cols());
  WorkspaceCoordinates wcs_coordinates = laser_coordinates / m;

  for (Eigen::Index i = 0; i < laser_coordinates.cols(); i++) {
    wcs_coordinates.col(i) = rotation * wcs_coordinates.col(i) + (translation / m);
    coordinates(0, i)      = focus_distance / wcs_coordinates(2, i) * wcs_coordinates(0, i);
    coordinates(1, i)      = focus_distance / wcs_coordinates(2, i) * wcs_coordinates(1, i);
  }

  return coordinates;
}

auto CameraModel::Undistort(const PlaneCoordinates& distorted, double K1, double K2, double K3, double P1, double P2)
    -> PlaneCoordinates {
  using Eigen::RowVector2d;

  const RowVectorXd rd2 =
      distorted.row(0).array() * distorted.row(0).array() + distorted.row(1).array() * distorted.row(1).array();
  const RowVectorXd rd4 = rd2.array() * rd2.array();
  const RowVectorXd rd6 = rd2.array() * rd2.array() * rd2.array();

  const RowVectorXd x_undistorted =
      distorted.row(0).array() * (K1 * rd2.array() + K2 * rd4.array() + K3 * rd6.array() + 1) +
      (P1 * (rd2.array() + 2 * distorted.row(0).array() * distorted.row(0).array())) +
      2 * P2 * distorted.row(0).array() * distorted.row(1).array();
  const RowVectorXd y_undistorted =
      distorted.row(1).array() * (K1 * rd2.array() + K2 * rd4.array() + K3 * rd6.array() + 1) +
      (2 * P1 * distorted.row(0).array() * distorted.row(1).array() +
       P2 * (distorted.row(0).array() * distorted.row(0).array() +
             2 * distorted.row(1).array() * distorted.row(1).array()));

  PlaneCoordinates undistorted(2, x_undistorted.cols());
  undistorted << x_undistorted.array(), y_undistorted.array();

  return undistorted;
}

auto CameraModel::Distort(const PlaneCoordinates& undistorted, double K1, double K2, double K3, double P1, double P2)
    -> PlaneCoordinates {
  using ceres::AutoDiffCostFunction;
  using ceres::Problem;
  using ceres::Solve;
  using ceres::Solver;
  using Eigen::Index;
  using Eigen::VectorXd;

  auto xu = new Xu;
  xu->K1  = K1;
  xu->K2  = K2;
  xu->K3  = K3;
  xu->P1  = P1;
  xu->P2  = P2;

  auto yu = new Yu;
  yu->K1  = K1;
  yu->K2  = K2;
  yu->K3  = K3;
  yu->P1  = P1;
  yu->P2  = P2;

  // initial guess
  double xd = 1.0;
  double yd = 1.0;

  Problem problem;
  problem.AddResidualBlock(new AutoDiffCostFunction<Xu, 1, 1, 1>(xu), nullptr, &xd, &yd);
  problem.AddResidualBlock(new AutoDiffCostFunction<Yu, 1, 1, 1>(yu), nullptr, &xd, &yd);

  Solver::Options options;
  options.max_num_iterations           = 1000;
  options.linear_solver_type           = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;

  PlaneCoordinates distorted = PlaneCoordinates::Zero(2, undistorted.cols());
  for (Index i = 0; i < undistorted.cols(); i++) {
    xu->xu = undistorted(0, i);
    yu->yu = undistorted(1, i);
    // xd = 1.0;
    // yd = 1.0;

    Solver::Summary summary;
    Solve(options, &problem, &summary);

    distorted(0, i) = xd;
    distorted(1, i) = yd;
  }

  return distorted;
}
}  // namespace scanner::image

// Error code implementation
namespace {

struct ErrorCategory : std::error_category {
  auto name() const noexcept -> const char* final;          // NOLINT(*-use-nodiscard)
  auto message(int error_code) const -> std::string final;  // NOLINT(*-use-nodiscard)
  auto default_error_condition(int other) const noexcept    // NOLINT(*-use-nodiscard)
      -> std::error_condition final;                        // NOLINT(*-use-nodiscard)
};

auto ErrorCategory::name() const noexcept -> const char* { return "CameraModelError"; }

auto ErrorCategory::message(int error_code) const -> std::string {
  switch (static_cast<scanner::image::CameraModelErrorCode>(error_code)) {
    case scanner::image::CameraModelErrorCode::NO_ERROR:
      return "No error";
    case scanner::image::CameraModelErrorCode::INVALID_IMAGE_COORDINATE_DIMENSION:
      return "Invalid image coordinate dimension";
    case scanner::image::CameraModelErrorCode::INVALID_WORKSPACE_COORDINATE_DIMENSION:
      return "Invalid workspace coordinate dimension";
  }
}

auto ErrorCategory::default_error_condition(int other) const noexcept -> std::error_condition {
  switch (static_cast<scanner::image::CameraModelErrorCode>(other)) {
    default:
      return {other, *this};
  }
}

const ErrorCategory ERROR_CATEGORY{};

}  // namespace

namespace scanner::image {

[[maybe_unused]] auto make_error_code(CameraModelErrorCode error_code) -> std::error_code {  // NOLINT(*-identifier-naming)
  return {static_cast<int>(error_code), ERROR_CATEGORY};
}

}  // namespace scanner::image
