#include "core/image/camera_model.h"

#include <ceres/ceres.h>  // IWYU pragma: keep

#include <Eigen/Core>
#include <string>
#include <system_error>

using core::image::CameraModel;
using core::image::CameraModelErrorCode;
using core::image::PlaneCoordinates;
using core::image::RotationMatrix;
using core::image::TranslationVector;
using core::image::WorkspaceCoordinates;
using Eigen::RowVectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;

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

auto CameraModel::Undistort(const core::image::PlaneCoordinates& distorted, double K1, double K2, double K3, double P1,
                            double P2) -> PlaneCoordinates {
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

auto CameraModel::Distort(const core::image::PlaneCoordinates& undistorted, double K1, double K2, double K3, double P1,
                          double P2) -> PlaneCoordinates {
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
  switch (static_cast<CameraModelErrorCode>(error_code)) {
    case CameraModelErrorCode::NO_ERROR:
      return "No error";
  }
}

auto ErrorCategory::default_error_condition(int other) const noexcept -> std::error_condition {
  switch (static_cast<CameraModelErrorCode>(other)) {
    default:
      return {other, *this};
  }
}

const ErrorCategory ERROR_CATEGORY{};

}  // namespace

[[maybe_unused]] auto core::image::make_error_code(CameraModelErrorCode error_code)
    -> std::error_code {  // NOLINT(*-identifier-naming)
  return {static_cast<int>(error_code), ERROR_CATEGORY};
}

#ifndef DOCTEST_CONFIG_DISABLE
// NOLINTBEGIN(readability-magic-number)

#include <doctest/doctest.h>

#include <cfloat>
#include <cstdlib>

TEST_SUITE("Test camera model helper functions") {
  class CameraModelExposed : public CameraModel {
   public:
    using CameraModel::Distort;
    using CameraModel::ImagePlaneToLaserPlane;
    using CameraModel::LaserPlaneToImagePlane;
    using CameraModel::ScaleFromPixels;
    using CameraModel::ScaleToPixels;
    using CameraModel::Undistort;
  };

  TEST_CASE("Test 1:1 scale") {
    Vector2d pixel_pitch;
    pixel_pitch << 1.0, 1.0;

    Vector2d principal_point;
    principal_point << 0.0, 0.0;

    PlaneCoordinates pixel_coordinates(2, 2);
    pixel_coordinates << 1.0, 2.0, 3.0, 4.0;

    auto coordinates = CameraModelExposed::ScaleFromPixels(pixel_coordinates, principal_point, pixel_pitch);

    CHECK_EQ(coordinates(0, 0), 1.0);
    CHECK_EQ(coordinates(0, 1), 2.0);
    CHECK_EQ(coordinates(1, 0), 3.0);
    CHECK_EQ(coordinates(1, 1), 4.0);
  }

  TEST_CASE("Test 1:2 scale") {
    Vector2d pixel_pitch;
    pixel_pitch << 0.5, 0.5;

    Vector2d principal_point;
    principal_point << 0.0, 0.0;

    PlaneCoordinates pixel_coordinates(2, 2);
    pixel_coordinates << 1.0, 2.0, 3.0, 4.0;

    auto coordinates = CameraModelExposed::ScaleFromPixels(pixel_coordinates, principal_point, pixel_pitch);

    CHECK_EQ(coordinates(0, 0), 0.5);
    CHECK_EQ(coordinates(0, 1), 1.0);
    CHECK_EQ(coordinates(1, 0), 1.5);
    CHECK_EQ(coordinates(1, 1), 2.0);
  }

  TEST_CASE("Test scale to length and back again") {
    Vector2d pixel_pitch;
    pixel_pitch << 0.5, 0.5;

    Vector2d principal_point;
    principal_point << 0.0, 0.0;

    PlaneCoordinates pixel_coordinates(2, 2);
    pixel_coordinates << 1.0, 2.0, 3.0, 4.0;

    auto coordinates = CameraModelExposed::ScaleFromPixels(pixel_coordinates, principal_point, pixel_pitch);
    coordinates      = CameraModelExposed::ScaleToPixels(coordinates, principal_point, pixel_pitch);

    CHECK_LE(abs(coordinates(0, 0) - 1.0), DBL_EPSILON);
    CHECK_LE(abs(coordinates(0, 1) - 2.0), DBL_EPSILON);
    CHECK_LE(abs(coordinates(1, 0) - 3.0), DBL_EPSILON);
    CHECK_LE(abs(coordinates(1, 1) - 4.0), DBL_EPSILON);
  }

  TEST_CASE("Removing distortion") {
    PlaneCoordinates coordinates(2, 5);
    coordinates << 1.0, 2.0, 3.0, 4.0, 5.0, 0.1, 0.2, 0.3, 0.4, 0.5;

    auto K1 = 3.780014812946319580e-03;
    auto K2 = -1.993117621168494225e-03;
    auto K3 = 5.228068857832113281e-07;
    auto P1 = -1.876385213108733296e-04;
    auto P2 = -5.847600405104458332e-04;

    auto K1c = 0.1;
    auto K2c = 0.1;
    auto K3c = 0.1;
    auto P1c = 0.1;
    auto P2c = 0.1;

    PlaneCoordinates undistorted =
        CameraModelExposed::Undistort(coordinates, K1 * K1c, K2 * K2c, K3 * K3c, P1 * P1c, P2 * P2c);

    // The target values are calculated with the Julia scripts in tests/math.
    CHECK_LE(abs(abs(undistorted(0, 0)) - 1.0001103430366931), DBL_EPSILON);
    CHECK_LE(abs(abs(undistorted(0, 1)) - 1.9962822753730811), DBL_EPSILON);
    CHECK_LE(abs(abs(undistorted(0, 2)) - 2.9604060766168758), DBL_EPSILON);
    CHECK_LE(abs(abs(undistorted(0, 3)) - 3.8160281882313654), DBL_EPSILON);
    CHECK_LE(abs(abs(undistorted(0, 4)) - 4.4148579915193125), DBL_EPSILON);

    CHECK_LE(abs(abs(undistorted(1, 0)) - 0.0999544534486835), DBL_EPSILON);
    CHECK_LE(abs(abs(undistorted(1, 1)) - 0.1994019041173649), DBL_EPSILON);
    CHECK_LE(abs(abs(undistorted(1, 2)) - 0.2955313799668153), DBL_EPSILON);
    CHECK_LE(abs(abs(undistorted(1, 3)) - 0.3806975251433637), DBL_EPSILON);
    CHECK_LE(abs(abs(undistorted(1, 4)) - 0.4400712777772862), DBL_EPSILON);
  }

  TEST_CASE("Adding distortion") {
    PlaneCoordinates coordinates(2, 5);
    coordinates << 1.0, 2.0, 3.0, 4.0, 5.0, 1.0, 2.0, 3.0, 4.0, 5.0;
    coordinates /= 10.0;

    auto K1 = 3.780014812946319580e-03;
    auto K2 = -1.993117621168494225e-03;
    auto K3 = 5.228068857832113281e-07;
    auto P1 = -1.876385213108733296e-04;
    auto P2 = -5.847600405104458332e-04;

    // auto K1c = 0.1;
    // auto K2c = 0.1;
    // auto K3c = 0.1;
    // auto P1c = 0.1;
    // auto P2c = 0.1;

    auto K1c = 1;
    auto K2c = 1;
    auto K3c = 1;
    auto P1c = 1;
    auto P2c = 1;

    PlaneCoordinates distorted =
        CameraModelExposed::Distort(coordinates, K1 * K1c, K2 * K2c, K3 * K3c, P1 * P1c, P2 * P2c);

    // The target values are calculated with the Julia scripts in tests/math.
    // The limits here are more generous than DBL_EPSILON since we are dealing with numerical solving
    CHECK_LE(abs(abs(distorted(0, 0)) - 0.10001172296544464), 1.0e-8);
    CHECK_LE(abs(abs(distorted(0, 1)) - 0.20001887346116246), 1.0e-8);
    CHECK_LE(abs(abs(distorted(0, 2)) - 0.29998805854655997), 1.0e-8);
    CHECK_LE(abs(abs(distorted(0, 3)) - 0.3999050796776773), 1.0e-8);
    CHECK_LE(abs(abs(distorted(0, 4)) - 0.49978436925089526), 1.0e-8);
    CHECK_LE(abs(abs(distorted(1, 0)) - 0.10001966852239728), 1.0e-8);
    CHECK_LE(abs(abs(distorted(1, 1)) - 0.20005065484120582), 1.0e-8);
    CHECK_LE(abs(abs(distorted(1, 2)) - 0.30005954086840064), 1.0e-8);
    CHECK_LE(abs(abs(distorted(1, 3)) - 0.40003208939287227), 1.0e-8);
    CHECK_LE(abs(abs(distorted(1, 4)) - 0.4999827147414687), 1.0e-8);
  }

  TEST_CASE("Undistort -> Distort -> Undistort") {
    PlaneCoordinates coordinates(2, 5);
    coordinates << 1.0, 2.0, 3.0, 4.0, 5.0, 0.1, 0.2, 0.3, 0.4, 0.5;

    auto K1 = 3.780014812946319580e-03;
    auto K2 = -1.993117621168494225e-03;
    auto K3 = 5.228068857832113281e-07;
    auto P1 = -1.876385213108733296e-04;
    auto P2 = -5.847600405104458332e-04;

    auto K1c = 0.1;
    auto K2c = 0.1;
    auto K3c = 0.1;
    auto P1c = 0.1;
    auto P2c = 0.1;

    PlaneCoordinates undistorted =
        CameraModelExposed::Undistort(coordinates, K1 * K1c, K2 * K2c, K3 * K3c, P1 * P1c, P2 * P2c);

    PlaneCoordinates distorted =
        CameraModelExposed::Distort(undistorted, K1 * K1c, K2 * K2c, K3 * K3c, P1 * P1c, P2 * P2c);

    // The limits here are more generous than DBL_EPSILON since we are dealing with numerical solving
    CHECK_LE(abs(abs(distorted(0, 0)) - 1.0), 1.0e-4);
    CHECK_LE(abs(abs(distorted(0, 1)) - 2.0), 1.0e-4);
    CHECK_LE(abs(abs(distorted(0, 2)) - 3.0), 1.0e-4);
    CHECK_LE(abs(abs(distorted(0, 3)) - 4.0), 1.0e-4);
    CHECK_LE(abs(abs(distorted(0, 4)) - 5.0), 1.0e-4);

    CHECK_LE(abs(abs(distorted(1, 0)) - 0.1), 1.0e-4);
    CHECK_LE(abs(abs(distorted(1, 1)) - 0.2), 1.0e-4);
    CHECK_LE(abs(abs(distorted(1, 2)) - 0.3), 1.0e-4);
    CHECK_LE(abs(abs(distorted(1, 3)) - 0.4), 1.0e-4);
    CHECK_LE(abs(abs(distorted(1, 4)) - 0.5), 1.0e-4);
  }

  TEST_CASE("Test image plane to laser plane transform") {
    RotationMatrix rotation;
    rotation << 9.999974673412257431e-01, 2.039705193809659024e-03, 9.512696023625968975e-04, 0.000000000000000000e+00,
        4.226691551490259768e-01, -9.062840533108859065e-01, -2.250624609754632317e-03, 9.062817580026263364e-01,
        4.226680846722816187e-01;
    TranslationVector translation;
    translation << 0, 0, 4.087606157143235386e-01;

    PlaneCoordinates image_coordinates(2, 5);
    image_coordinates << 1.0, 2.0, 3.0, 4.0, 5.0, 0.1, 0.2, 0.3, 0.4, 0.5;

    auto laser_plane_coordinates = CameraModelExposed::ImagePlaneToLaserPlane(image_coordinates, rotation, translation,
                                                                              3.744565963745117188e+00, 0.1, 0.007093);

    CHECK_LE(abs(abs(laser_plane_coordinates(0, 0)) - 0.1156620887972415), DBL_EPSILON);
    CHECK_LE(abs(abs(laser_plane_coordinates(0, 1)) - 0.2461062539208822), DBL_EPSILON);
    CHECK_LE(abs(abs(laser_plane_coordinates(0, 2)) - 0.3943597558330662), DBL_EPSILON);
    CHECK_LE(abs(abs(laser_plane_coordinates(0, 3)) - 0.5643370316351004), DBL_EPSILON);
    CHECK_LE(abs(abs(laser_plane_coordinates(0, 4)) - 0.7611903864963705), DBL_EPSILON);

    CHECK_LE(abs(abs(laser_plane_coordinates(1, 0)) - 0.0273778290076216), DBL_EPSILON);
    CHECK_LE(abs(abs(laser_plane_coordinates(1, 1)) - 0.0582546537730602), DBL_EPSILON);
    CHECK_LE(abs(abs(laser_plane_coordinates(1, 2)) - 0.0933470428811987), DBL_EPSILON);
    CHECK_LE(abs(abs(laser_plane_coordinates(1, 3)) - 0.1335815643262275), DBL_EPSILON);
    CHECK_LE(abs(abs(laser_plane_coordinates(1, 4)) - 0.1801777960302591), DBL_EPSILON);
  }

  TEST_CASE("Test image plane to laser plane to image plane") {
    RotationMatrix rotation;
    rotation << 9.999974673412257431e-01, 2.039705193809659024e-03, 9.512696023625968975e-04, 0.000000000000000000e+00,
        4.226691551490259768e-01, -9.062840533108859065e-01, -2.250624609754632317e-03, 9.062817580026263364e-01,
        4.226680846722816187e-01;
    TranslationVector translation;
    translation << 0, 0, 4.087606157143235386e-01;

    double focus_distance = 3.744565963745117188e+00;

    PlaneCoordinates image_coordinates(2, 5);
    image_coordinates << 1.0, 2.0, 3.0, 4.0, 5.0, 0.1, 0.2, 0.3, 0.4, 0.5;

    auto laser_plane_coordinates = CameraModelExposed::ImagePlaneToLaserPlane(image_coordinates, rotation, translation,
                                                                              focus_distance, 0.1, 0.007093);

    auto image_coordinates_calculated =
        CameraModelExposed::LaserPlaneToImagePlane(laser_plane_coordinates, rotation, translation, focus_distance, 0.1);

    PlaneCoordinates diff = image_coordinates - image_coordinates_calculated;

    CHECK_LE(abs(diff(0, 0)), 4 * DBL_EPSILON);
    CHECK_LE(abs(diff(0, 1)), 4 * DBL_EPSILON);
    CHECK_LE(abs(diff(0, 2)), 4 * DBL_EPSILON);
    CHECK_LE(abs(diff(0, 3)), 4 * DBL_EPSILON);
    CHECK_LE(abs(diff(0, 4)), 4 * DBL_EPSILON);

    CHECK_LE(abs(diff(1, 0)), 4 * DBL_EPSILON);
    CHECK_LE(abs(diff(1, 1)), 4 * DBL_EPSILON);
    CHECK_LE(abs(diff(1, 2)), 4 * DBL_EPSILON);
    CHECK_LE(abs(diff(1, 3)), 4 * DBL_EPSILON);
    CHECK_LE(abs(diff(1, 4)), 4 * DBL_EPSILON);
  }
}

// NOLINTEND(readability-magic-number)
#endif
