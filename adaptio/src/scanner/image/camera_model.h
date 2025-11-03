#pragma once

#include <boost/outcome.hpp>
#include <Eigen/Eigen>

#include "image.h"

namespace scanner::image {

using PlaneCoordinates     = Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::RowMajor>;
using WorkspaceCoordinates = Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor>;
using TranslationVector    = Eigen::Vector3d;
using RotationMatrix       = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

enum class CameraModelErrorCode : uint32_t {
  NO_ERROR = 0,
  INVALID_IMAGE_COORDINATE_DIMENSION,
  INVALID_WORKSPACE_COORDINATE_DIMENSION,
};

// NOLINTNEXTLINE(*-identifier-naming)
[[maybe_unused]] auto make_error_code(CameraModelErrorCode) -> std::error_code;

class CameraProperties {};

class CameraModel {
 public:
  CameraModel()                                 = default;
  CameraModel(CameraModel&)                     = delete;
  auto operator=(CameraModel&) -> CameraModel&  = delete;
  CameraModel(CameraModel&&)                    = delete;
  auto operator=(CameraModel&&) -> CameraModel& = delete;

  virtual ~CameraModel() = default;

  /**
   * Calculates the coordinates of the laser line as workspace coordinates.
   *
   * @param image_coordinates The coordinates of the laser line in image space coordinates.
   * @param vertical_crop_offset Allows for dynamic resizing of the FOV vertically.
   * @return The workspace coordinates if calculations are successful.
   */
  virtual auto ImageToWorkspace(const PlaneCoordinates&, int, int horizontal_crop_offset = 0) const
      -> boost::outcome_v2::result<WorkspaceCoordinates> = 0;

  /**
   * Calculates the coordinates of the laser line as image space coordinates.
   *
   * @param workspace_coordinates The coordinates of the laser line in workspace coordinates.
   * @param vertical_crop_offset Allows for dynamic resizing of the FOV vertically.
   * @return The image space coordinates if calculations are successful.
   */
  virtual auto WorkspaceToImage(const WorkspaceCoordinates&, int, int horizontal_crop_offset = 0) const
      -> boost::outcome_v2::result<PlaneCoordinates> = 0;

 protected:
  /**
   * Uses the pixel pitch and principal point to scale the pixel coordinates into length units.
   * @param pixel_coordinates A 2 by n matrix with the coordinates arranged as rows [ [x...], [y...] ].
   * @param principal_point The principal point of the camera sensor.
   * @param pixel_pitch The pixel pitch of the camera sensor.
   * @return A 2 by n matrix with the scaled coordinates, the unit will depend on the unit of pixel pitch & principal
   * point.
   */
  static auto ScaleFromPixels(const PlaneCoordinates& pixel_coordinates, const Eigen::Vector2d& principal_point,
                              const Eigen::Vector2d& pixel_pitch) -> PlaneCoordinates;

  /**
   * Uses the pixel pitch and principal point to scale length units into pixel coordinates.
   * @param length_coordinates A 2 by n matrix with the coordinates arranged as rows [ [x...], [y...] ].
   * @param principal_point The principal point of the camera sensor.
   * @param pixel_pitch The pixel pitch of the camera sensor.
   * @return A 2 by n matrix with the scaled coordinates, the unit will depend on the unit of pixel pitch & principal
   * point.
   */
  static auto ScaleToPixels(const PlaneCoordinates& length_coordinates, const Eigen::Vector2d& principal_point,
                            const Eigen::Vector2d& pixel_pitch) -> PlaneCoordinates;

  /**
   * Transforms the given image plane coordinates (2D) into workspace coordinates (3D) (in the laser plane).
   * The coordinate system used here originates in this laser plane which gives Z = 0 for all points returned.
   *
   * @param image_coordinates
   * @param rotation
   * @param translation
   * @param focus_distance
   * @return
   */
  static auto ImagePlaneToLaserPlane(const PlaneCoordinates& image_coordinates, const RotationMatrix& rotation,
                                     const TranslationVector& translation, double focus_distance, double m, double w)
      -> WorkspaceCoordinates;

  /**
   * Transforms the given laser plane coordinates (3D) into image plane coordinates (2D).
   *
   * @param image_coordinates
   * @param rotation
   * @param translation
   * @param focus_distance
   * @return
   */
  static auto LaserPlaneToImagePlane(const WorkspaceCoordinates& laser_coordinates, const RotationMatrix& rotation,
                                     const TranslationVector& translation, double focus_distance, double m)
      -> PlaneCoordinates;

  /**
   * Undistorts the given coordinates according to the given coefficients.
   * The coefficients will be given by the camera calibration.
   *
   * This uses equation 12 from https://link.springer.com/content/pdf/10.1007/s11263-016-0964-8.pdf:
   * xu = xd * (1 + K1 * rd^2 + K2 * rd^4 + K3 * rd^6) + P1 * (rd + 2*xd^2) + 2 * P2 * xd * yd
   * yu = yd * (1 + K1 * rd^2 + K2 * rd^4 + K3 * rd^6) + (2 * P2 * xd * yd + P2 * (rd^2 + 2 * yd^2))
   *
   * @param distorted
   * @param K1
   * @param K2
   * @param K3
   * @param P1
   * @param P2
   * @return
   */
  static auto Undistort(const PlaneCoordinates& distorted, double K1, double K2, double K3, double P1, double P2)
      -> PlaneCoordinates;

  /**
   * Distorts the given coordinates according to the given coefficients.
   * The coefficients will be given by the camera calibration.
   *
   * This uses equation 12 from https://link.springer.com/content/pdf/10.1007/s11263-016-0964-8.pdf:
   * xu = xd * (1 + K1 * rd^2 + K2 * rd^4 + K3 * rd^6) + P1 * (rd + 2*xd^2) + 2 * P2 * xd * yd
   * yu = yd * (1 + K1 * rd^2 + K2 * rd^4 + K3 * rd^6) + (2 * P2 * xd * yd + P2 * (rd^2 + 2 * yd^2))
   *
   * Since these functions are not easily invertable we use numerical methods to find the roots.
   *
   * @param undistorted
   * @param K1
   * @param K2
   * @param K3
   * @param P1
   * @param P2
   * @return
   */
  static auto Distort(const PlaneCoordinates& undistorted, double K1, double K2, double K3, double P1, double P2)
      -> PlaneCoordinates;

 private:
  struct Xu {
    double xu;
    double K1;
    double K2;
    double K3;
    double P1;
    double P2;

    template <typename T>
    bool operator()(const T* const xd, const T* const yd, T* residual) const {
      auto rd2 = xd[0] * xd[0] + yd[0] * yd[0];
      auto rd4 = rd2 * rd2;
      auto rd6 = rd4 * rd2;

      residual[0] = xd[0] * (1.0 + K1 * rd2 + K2 * rd4 + K3 * rd6) + P1 * (rd2 + 2.0 * xd[0] * xd[0]) +
                    2.0 * P2 * xd[0] * yd[0] - xu;
      return true;
    }
  };

  struct Yu {
    double yu;
    double K1;
    double K2;
    double K3;
    double P1;
    double P2;

    template <typename T>
    bool operator()(const T* const xd, const T* const yd, T* residual) const {
      auto rd2 = xd[0] * xd[0] + yd[0] * yd[0];
      auto rd4 = rd2 * rd2;
      auto rd6 = rd4 * rd2;

      residual[0] = yd[0] * (1.0 + K1 * rd2 + K2 * rd4 + K3 * rd6) + P2 * (rd2 + 2.0 * yd[0] * yd[0]) +
                    2.0 * P1 * xd[0] * yd[0] - yu;
      return true;
    }
  };
};

using CameraModelPtr = std::unique_ptr<CameraModel>;

}  // namespace scanner::image

namespace std {
template <>
struct is_error_code_enum<scanner::image::CameraModelErrorCode> : true_type {};
}  // namespace std
