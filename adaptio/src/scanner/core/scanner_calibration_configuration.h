#pragma once

#include <fmt/format.h>

#include <cstdint>
#include <eigen3/Eigen/Eigen>
#include <memory>
#include <string>

namespace scanner {

struct ScannerCalibrationData {
  std::string scanner_serial_number;
  /**
   * Contains a calibrated matrix with the intrinsic properties of the camera.
   * TODO: Properly document
   */
  struct {
    // The distance from the image sensor to the object side projection center
    double projection_center_distance;
    double focus_distance;

    // Principal point (optical center) of the image sensor
    struct {
      double x;
      double y;
    } principal_point;

    // Pixel pitch of the sensor
    struct {
      double x;
      double y;
    } pixel_pitch;

    double rho;
    double tau;
    double d;
    double K1;  // NOLINT
    double K2;  // NOLINT
    double K3;  // NOLINT
    double P1;  // NOLINT
    double P2;  // NOLINT

    // These scaling factors are kept here for posterity,
    // they should already be applied in the proper places above.
    // The factors are calculated by the optimizer when calibrating the scanner.
    struct {
      double w;  // Scaling factor for image sensor?
      double m;  // Length scaling factor?
      double K1;
      double K2;
      double K3;
      double P1;
      double P2;
    } scaling_factors;
  } intrinsic;

  /**
   * Struct that contains calibrated matrices/vectors used to transform image
   * coordinates
   */
  struct {
    /** Rotation matrix */
    Eigen::Matrix3d rotation;  // NOLINT
    /** Translation column vector */
    Eigen::Vector3d translation;
  } extrinsic;
};

inline auto MatrixToString(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>& matrix)
    -> std::string {
  std::string res = "[";
  for (int i = 0; i < matrix.rows(); i++) {
    for (int j = 0; j < matrix.cols(); j++) {
      res += fmt::to_string(matrix(i, j));
      if (i != matrix.rows() - 1 || j != matrix.cols() - 1) {
        res += ", ";
      }
    }
  }

  res += "]";
  return res;
}

inline auto ToYaml(const ScannerCalibrationData& data) -> std::string {
  return "\nscanner_serial_number: " + data.scanner_serial_number + "\n" + "intrinsic:\n" +
         "  projection_center_distance: " + fmt::format("{:1.f}", data.intrinsic.projection_center_distance) + "\n" +
         "  focus_distance: " + fmt::to_string(data.intrinsic.focus_distance) + "\n" + "  principal_point: \n" +
         "    x: " + fmt::to_string(data.intrinsic.principal_point.x) + "\n" +
         "    y: " + fmt::to_string(data.intrinsic.principal_point.y) + "\n" + "  pixel_pitch: \n" +
         "    x: " + fmt::to_string(data.intrinsic.pixel_pitch.x) + "\n" +
         "    y: " + fmt::to_string(data.intrinsic.pixel_pitch.y) + "\n" +
         "  rho: " + fmt::to_string(data.intrinsic.rho) + "\n" + "  tau: " + fmt::to_string(data.intrinsic.tau) + "\n" +
         "  d: " + fmt::to_string(data.intrinsic.d) + "\n" + "  K1: " + fmt::to_string(data.intrinsic.K1) + "\n" +
         "  K2: " + fmt::to_string(data.intrinsic.K2) + "\n" + "  K3: " + fmt::to_string(data.intrinsic.K3) + "\n" +
         "  P1: " + fmt::to_string(data.intrinsic.P1) + "\n" + "  P2: " + fmt::to_string(data.intrinsic.P2) + "\n" +
         "  scaling_factors: \n" + "    w: " + fmt::to_string(data.intrinsic.scaling_factors.w) + "\n" +
         "    m: " + fmt::to_string(data.intrinsic.scaling_factors.m) + "\n" +
         "    K1: " + fmt::to_string(data.intrinsic.scaling_factors.K1) + "\n" +
         "    K2: " + fmt::to_string(data.intrinsic.scaling_factors.K2) + "\n" +
         "    K3: " + fmt::to_string(data.intrinsic.scaling_factors.K3) + "\n" +
         "    P1: " + fmt::to_string(data.intrinsic.scaling_factors.P1) + "\n" +
         "    P2: " + fmt::to_string(data.intrinsic.scaling_factors.P2) + "\n" +

         "extrinsic:\n" + "  R: !matrix\n" + "    rows: " + fmt::to_string(data.extrinsic.rotation.rows()) + "\n" +
         "    columns: " + fmt::to_string(data.extrinsic.rotation.cols()) + "\n" +
         "    data: " + MatrixToString(data.extrinsic.rotation) + "\n" + "  t: !matrix\n" +
         "    rows: " + fmt::to_string(data.extrinsic.translation.rows()) + "\n" +
         "    columns: " + fmt::to_string(data.extrinsic.translation.cols()) + "\n" +
         "    data: " + MatrixToString(data.extrinsic.translation) + "\n";
}

}  // namespace scanner
