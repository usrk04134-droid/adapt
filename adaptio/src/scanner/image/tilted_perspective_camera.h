#pragma once

#include <Eigen/Eigen>

#include "common/data/data_value.h"
#include "scanner/image/camera_model.h"
#include "scanner/image_provider/image_provider_configuration.h"
#include "scanner/scanner_calibration_configuration.h"

namespace scanner::image {

/*
 * TODO: Rename this to something like CalibrationParameters or OptimizedParameters to signify that they originate from
 * the scanner calibration.
 */
class TiltedPerspectiveCameraProperties : public CameraProperties {
 public:
  scanner::ScannerCalibrationData config_calib;
  image_provider::Fov config_fov;

  static auto FromUnorderedMap(const std::unordered_map<std::string, common::data::DataValue> &map)
      -> TiltedPerspectiveCameraProperties;
};

/**
 * The math here is based on the tilted camera model with perspective lens described in
 * https://link.springer.com/content/pdf/10.1007/s11263-016-0964-8.pdf
 */
class TiltedPerspectiveCamera : public CameraModel {
 public:
  explicit TiltedPerspectiveCamera(const TiltedPerspectiveCameraProperties &camera_properties);

  TiltedPerspectiveCamera(TiltedPerspectiveCamera &)                      = delete;
  auto operator=(TiltedPerspectiveCamera &) -> TiltedPerspectiveCamera &  = delete;
  TiltedPerspectiveCamera(TiltedPerspectiveCamera &&)                     = delete;
  auto operator=(TiltedPerspectiveCamera &&) -> TiltedPerspectiveCamera & = delete;

  ~TiltedPerspectiveCamera() override = default;

  auto ImageToWorkspace(const PlaneCoordinates &image_coordinates, int vertical_crop_offset) const
      -> boost::outcome_v2::result<WorkspaceCoordinates> override;

  auto WorkspaceToImage(const WorkspaceCoordinates &workspace_coordinates, int vertical_crop_offset) const
      -> boost::outcome_v2::result<PlaneCoordinates> override;

  // Add dynamic horizontal ROI offset (in pixels) to configured FOV offset_x
  void SetDynamicHorizontalOffsetPixels(int64_t offset_pixels) override {
    dynamic_offset_x_pixels_ = static_cast<int>(offset_pixels);
  }

  /**
   * Sets the camera properties and recalculates relevant matrices.
   * @param camera_properties
   */
  void SetCameraProperties(const TiltedPerspectiveCameraProperties &camera_properties);

  // Dynamically adjust horizontal FOV offset relative to configured FOV.
  // This value represents the additional OffsetX (in pixels) applied by the
  // camera at runtime, i.e. the hardware ROI offset from the configured
  // image_provider::Fov.offset_x.
  void SetHorizontalFovRelativeOffset(int64_t offset_pixels) { dynamic_offset_x_pixels_ = offset_pixels; }

  void SetDynamicHorizontalOffsetPixels(int64_t offset_pixels) override { SetHorizontalFovRelativeOffset(offset_pixels); }

  /**
   * Gets the current matrix used to perform the tilt transformation.
   * @return A 3x3 matrix
   */
  auto GetTiltTransformationMatrix() -> Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

 private:
  TiltedPerspectiveCameraProperties camera_properties_;
  Eigen::Matrix<double, 3, 3, Eigen::RowMajor> Hp_;
  int dynamic_offset_x_pixels_ = 0;  // additional OffsetX applied at runtime
  // Runtime horizontal ROI offset in pixels, applied in addition to
  // camera_properties_.config_fov.offset_x for coordinate transforms.
  int64_t dynamic_offset_x_pixels_ = 0;

  /**
   * Calculates the tilt transformation matrix Hp for a tilted lens camera model with an image-side perspective lens.
   *
   * @param rho
   * @param tau
   * @param d
   * @return
   */
  static auto CalculateTiltTransformationMatrix(double rho, double tau, double d)
      -> Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
};

}  // namespace scanner::image
