#pragma once

#include <cmath>
#include <Eigen/Eigen>
#include <optional>
#include <utility>
#include <vector>

#include "calibration/calibration_types.h"
#include "calibration/src/calibration_solver.h"
#include "common/groove/point.h"
#include "slice_translator/model_extract.h"
#include "slice_translator/src/model_impl.h"

namespace calibration {

using Eigen::Matrix3d;
using Eigen::Vector3d;

template <typename GenScalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor {
  using Scalar = GenScalar;
  enum { InputsAtCompileTime = NX, ValuesAtCompileTime = NY };  // NOLINT
  using InputType    = Eigen::Matrix<Scalar, InputsAtCompileTime, 1>;
  using ValueType    = Eigen::Matrix<Scalar, ValuesAtCompileTime, 1>;
  using JacobianType = Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime>;

  int m_inputs, m_values;

  Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
  Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}  // NOLINT
  Functor(int inputs) : m_inputs(inputs) {}                                // NOLINT

  auto inputs() const -> int { return m_inputs; }  // NOLINT
  auto values() const -> int { return m_values; }  // NOLINT
};

struct ClockPosFunctor : public Functor<double> {
  using Scalar = Functor::Scalar;
  slice_translator::ModelExtract *model_extract;
  double scanner_mount_angle;
  // Eigen::MatrixX3d slide_positions_mcs;
  // Eigen::MatrixX3d abwx_points_lpcs;
  std::vector<Observation> observations;
  TorchPlaneInfo torch_plane_info;
  // Eigen::Matrix3d R_10to4;  // NOLINT R_10to4 == R_1to4 == R_3to4
  double object_radius{NAN};
  double ltc_stickout{NAN};
  double ltc_torch_plane_dist{NAN};

  explicit ClockPosFunctor(int nbr_model_params);

  auto operator()(const Eigen::VectorXd &params, Eigen::VectorXd &fvec) const -> int;
  // auto ComputeDistanceFromTorchToLaserPlane(double clock_pos_laser, double clock_pos_torch, const Vector3d &abwx_4,
  //                                           const Vector3d &slide_pos, const Matrix3d &R_10to11) const -> double;

  auto ComputeTorchPlaneClockPosition(const Matrix3d &R_10to11, const Matrix3d &R_3to4, double laser_clockpos,
                                      const Vector3d &a_4to10) const -> double;
  auto ComputeTcsToLpcsTranslation(const Matrix3d &R_10to11, const Matrix3d &R_10to4, double laser_clockpos,
                                   double torch_clockpos, const Vector3d &a_4to10) const -> Eigen::Vector3d;
  auto ComputeRotationCenter(const Matrix3d &R_10to11, double clock_pos_laser, double clock_pos_torch) const
      -> Eigen::Vector3d;
  auto ComputeWeldObjectOrientation(double rotation_angle) const -> Matrix3d;
  // auto inputs() const -> int { return 6; }                          // Number of parameters in the model
  auto values() const -> int { return this->observations.size(); }  // The number of observations
  // auto values() const -> int { return this->abwx_points_lpcs.rows(); }  // The number of observations
};

class CalibrationSolverImpl : public CalibrationSolver {
 public:
  explicit CalibrationSolverImpl(slice_translator::ModelExtract *model_extract);
  auto Calculate(const TorchPlaneInfo &torch_plane_info, const GeometricConstants &geometric_constants,
                 const std::vector<calibration::Observation> &observations)
      -> std::optional<CalibrationResult> override;

 private:
  slice_translator::ModelExtract *model_extract_;
  ClockPosFunctor clock_pos_functor_;
  auto static CreateRotationMatrix(double angle, Eigen::Vector3d &rot_axis) -> Eigen::Matrix3d;
  auto static ComputeStartValues(const std::vector<calibration::Observation> &observations,
                                 const Eigen::Vector2d &center_slide_pos, double object_radius) -> Eigen::VectorXd;
  auto SolveClockPosition(const GeometricConstants &geometric_constants, const TorchPlaneInfo &torch_plane_info,
                          const std::vector<Observation> &observations) -> std::pair<bool, Eigen::VectorXd>;
  auto ComputeModelQuality(CalibrationResult &result, const std::vector<Observation> &observations,
                           const GeometricConstants &constants, const common::groove::Point ref_point_macs) const -> void;
};
}  // namespace calibration
