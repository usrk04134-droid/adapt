#include "calibration_solver_impl.h"

#include "common/logging/application_log.h"
// #include <unsupported/Eigen/src/NonLinearOptimization/LevenbergMarquardt.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <numbers>
#include <optional>
#include <sstream>
#include <string>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
#include <utility>
#include <vector>

#include "calibration_solver.h"
#include "common/types/vector_3d.h"
#include "common/types/vector_3d_helpers.h"
#include "lpcs/lpcs_point.h"
#include "macs/macs_point.h"
#include "slice_translator/model_extract.h"

namespace {
auto ToString(const Eigen::VectorXd &value) -> std::string {
  std::ostringstream oss;
  oss << value.transpose();  // more compact (row form)
  return oss.str();
}
}  // namespace

using calibration::CalibrationSolverImpl;
using calibration::ClockPosFunctor;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::LevenbergMarquardtSpace::Status;

// namespace calibration {
const double INIT_LASER_CLOCK_POS = 15 * std::numbers::pi / 180;
const int PRIMARY_ABW             = 0;
const int MAX_ITER                = 10000;
const int TOP_LEFT_ABW_IND        = 0;
const int TOP_RIGHT_ABW_IND       = 6;
const int LASER_CLOCK_PARAM_IDX   = 0;
const int AX_PARAM_IDX            = 1;
const int AY_PARAM_IDX            = 2;
const int OBJECT_ROT_PARAM_IDX    = 3;
const int DELTA_Y_PARAM_IDX       = 4;
const int DELTA_Z_PARAM_IDX       = 5;
const bool OPTIMIZE_ORIENTATIONS  = false;
const int NBR_MODEL_PARAMS        = OPTIMIZE_ORIENTATIONS ? 6 : 3;

ClockPosFunctor::ClockPosFunctor(int nbr_model_params) : Functor(nbr_model_params) {}
// NOLINTBEGIN(*readability-identifier-naming*)
auto ClockPosFunctor::operator()(const Eigen::VectorXd &params, Eigen::VectorXd &fvec) const -> int {
  // Extract optimization parameters
  const double clock_pos_laser = params(LASER_CLOCK_PARAM_IDX);
  const double ax              = params(AX_PARAM_IDX);
  const double ay              = params(AY_PARAM_IDX);
  const double obj_angle       = OPTIMIZE_ORIENTATIONS ? params(OBJECT_ROT_PARAM_IDX) : 0.0;
  const double delta_rot_y     = OPTIMIZE_ORIENTATIONS ? params(DELTA_Y_PARAM_IDX) : 0.0;
  const double delta_rot_z     = OPTIMIZE_ORIENTATIONS ? params(DELTA_Z_PARAM_IDX) : 0.0;
  const Vector3d a_4to10{ax, ay, 0.0};  // t_4to10 for the ref slide pos.

  // Create the transformation matrices
  Matrix3d R_10to4  = model_extract->ComputeLpcsOrientation(scanner_mount_angle, delta_rot_y, delta_rot_z);
  Matrix3d R_10to11 = ComputeWeldObjectOrientation(obj_angle);  // Rot around MACS z
  Matrix3d R_11to4  = R_10to11.transpose() * R_10to4;

  // Compute the clock position of torch when at reference point (top center)
  const double clock_pos_torch = ComputeTorchPlaneClockPosition(R_10to11, R_10to4, clock_pos_laser, a_4to10);

  // Translation of LPCS relative to VTCS when torch is at reference point (top center) in torch plane.
  const Vector3d a_10to4 = -R_10to4 * a_4to10;

  // Tranlation of LPCS rel TCS
  const Vector3d t_3to4 = ComputeTcsToLpcsTranslation(R_10to11, R_10to4, clock_pos_laser, clock_pos_torch, a_4to10);

  // Weld rotation axis (can have any orientation if rotation matrix is computed from measurement in future)
  const Vector3d rotax_11 = {1.0, 0.0, 0.0};
  const Vector3d rotax_10 = R_10to11 * rotax_11;

  Eigen::Vector c_1 = ComputeRotationCenter(R_10to11, clock_pos_laser, clock_pos_torch);

  // Convert to common::vector for calls to model
  const common::Vector3D tcs2lpcs   = common::EigenVector2CommonVector(t_3to4);
  const common::Vector3D rot_axis   = common::EigenVector2CommonVector(rotax_10);
  const common::Vector3D rot_center = common::EigenVector2CommonVector(c_1);

  int obs_index         = 0;
  macs::Point ref_point = torch_plane_info.top_center_at_torch_plane;

  for (const auto &observation : observations) {
    // Using top center point as rotaional correspondence for optimization
    lpcs::Point point_lpcs{
        .x =
            (observation.abw_points_lpcs.at(TOP_LEFT_ABW_IND).x + observation.abw_points_lpcs.at(TOP_RIGHT_ABW_IND).x) /
            2,
        .y =
            (observation.abw_points_lpcs.at(TOP_LEFT_ABW_IND).y + observation.abw_points_lpcs.at(TOP_RIGHT_ABW_IND).y) /
            2};

    // Call the model to transform and rotate/project lpcs point --> mcs
    macs::Point computed_point_mcs =
        this->model_extract->TransformAndRotateToTorchPlane(rot_center, {scanner_mount_angle, delta_rot_y, delta_rot_z},
                                                            rot_axis, tcs2lpcs, point_lpcs, observation.slide_position);

    // Compute the errors
    // Here we are using the norm of the error vector (which has two components MACS x an MACS z)
    // The LM algoritm will square the error and that should not be done here.
    fvec(obs_index) = (computed_point_mcs - ref_point).Norm();
    obs_index++;
  }

  return 0;
}

auto ClockPosFunctor::ComputeWeldObjectOrientation(double rotation_angle) const -> Matrix3d {
  Vector3d rotax_10 = {0.0, 0.0, 1.0};
  Matrix3d rot_z    = Eigen::AngleAxisd(rotation_angle, rotax_10).toRotationMatrix();
  Matrix3d R_10to11 = rot_z;  // rot_y * rot_z; //Extrinsic
  return R_10to11;
}

auto ClockPosFunctor::ComputeRotationCenter(const Matrix3d &R_10to11, double clock_pos_laser,
                                            double clock_pos_torch) const -> Eigen::Vector3d {
  // Vectors from rotation center to laser (ra_*) and torch (rb_*)
  const Vector3d ra_11 = {0.0, object_radius * std::sin(clock_pos_laser), object_radius * std::cos(clock_pos_laser)};
  const Vector3d rb_11 = {0.0, object_radius * std::sin(clock_pos_torch), object_radius * std::cos(clock_pos_torch)};

  // Reference point in torch plane
  const Vector3d f_11 = rb_11 - ra_11;
  const Vector3d f_10 = R_10to11 * f_11;
  const Vector3d g_1  = {torch_plane_info.top_center_at_torch_plane.horizontal, 0.0,
                         torch_plane_info.top_center_at_torch_plane.vertical};

  // Translation vector VTCS to MACS
  const Vector3d t_10to1 = f_10 - g_1;  // VTCS and MACS same orientation => no rot transformation.

  // Projection/rotation axis (for this iteration of clock pos)
  const Vector3d c_11{0, -object_radius * std::sin(clock_pos_laser), -object_radius * std::cos(clock_pos_laser)};
  const Vector3d c_1 = R_10to11 * c_11 - t_10to1;

  return c_1;
}

auto ClockPosFunctor::ComputeTorchPlaneClockPosition(const Matrix3d &R_10to11, const Matrix3d &R_3to4,
                                                     double laser_clockpos, const Vector3d &a_4to10) const -> double {
  Matrix3d R_10to4 = R_3to4;

  // Measured distance from wire tip to laser plane (LTC)
  const double t = ltc_torch_plane_dist;

  // Laser plane normal
  Vector3d n_4  = {0, 0, 1};
  Vector3d n_10 = R_10to4 * n_4;
  Vector3d n_11 = R_10to11.transpose() * n_10;

  // Direction along which torch to laserplane distance is measured
  Vector3d d_10 = {0, 1, 0};

  // Stickout vector
  Vector3d s_10 = {0, 0, -ltc_stickout};

  // Vectors from rotation center to laser (ra_*)
  Vector3d ra_11 = {0, object_radius * std::sin(laser_clockpos), object_radius * std::cos(laser_clockpos)};
  Vector3d ra_10 = R_10to11 * ra_11;

  // Measured ref point in torch plane.
  Vector3d g_1 = {torch_plane_info.top_center_at_torch_plane.horizontal, 0.0,
                  torch_plane_info.top_center_at_torch_plane.vertical};

  // Constant in equation to solve K = b_10*n_11
  double K = (-t * d_10 - s_10 + ra_10).dot(n_10);  // / d_10.dot(n_10);

  // Solve clock position
  //
  // Since b_11(0) = 0 by defintion we only need to consider components 1 and 2
  double n                 = object_radius * n_11(2);  // cos
  double m                 = object_radius * n_11(1);  // sin
  double c                 = (n < 0.0 ? -1.0 : 1.0) * std::sqrt(m * m + n * n);
  double phi               = std::atan(-m / n);
  double torch_clock_angle = std::acos(K / c) - phi;

  return torch_clock_angle;
}

auto ClockPosFunctor::ComputeTcsToLpcsTranslation(const Matrix3d &R_10to11, const Matrix3d &R_10to4,
                                                  double laser_clockpos, double torch_clockpos,
                                                  const Vector3d &a_4to10) const -> Eigen::Vector3d {
  // R_10to4 == R_3to4
  const Vector3d ra_11  = {0, object_radius * std::sin(laser_clockpos), object_radius * std::cos(laser_clockpos)};
  const Vector3d rb_11  = {0, object_radius * std::sin(torch_clockpos), object_radius * std::cos(torch_clockpos)};
  const Vector3d t_3to4 = -R_10to4 * a_4to10 - R_10to11 * rb_11 + R_10to11 * ra_11;
  return t_3to4;
}

CalibrationSolverImpl::CalibrationSolverImpl(slice_translator::ModelExtract *model_extract)
    : model_extract_(model_extract), clock_pos_functor_(NBR_MODEL_PARAMS) {}

auto CalibrationSolverImpl::Calculate(const TorchPlaneInfo &torch_plane_info,
                                      const GeometricConstants &geometric_constants,
                                      const std::vector<Observation> &observations)
    -> std::optional<CalibrationResult> {
  const std::pair<bool, Eigen::VectorXd> opt_result =
      SolveClockPosition(geometric_constants, torch_plane_info, observations);

  if (!opt_result.first) {
    return {};  // Failed
  }

  // Create variables for further calculations from the optimized parameters.
  Eigen::VectorXd solved_params = opt_result.second;
  const Vector3d a_4to10        = {solved_params(AX_PARAM_IDX), solved_params(AY_PARAM_IDX), 0};
  const double clock_pos_laser  = solved_params(LASER_CLOCK_PARAM_IDX);
  const double obj_angle        = OPTIMIZE_ORIENTATIONS ? solved_params(OBJECT_ROT_PARAM_IDX) : 0.0;
  const double delta_rot_y      = OPTIMIZE_ORIENTATIONS ? solved_params(DELTA_Y_PARAM_IDX) : 0.0;
  const double delta_rot_z      = OPTIMIZE_ORIENTATIONS ? solved_params(DELTA_Z_PARAM_IDX) : 0.0;

  // Misalignment of weld object.
  const Matrix3d R_10to11 = clock_pos_functor_.ComputeWeldObjectOrientation(obj_angle);

  const Matrix3d R_3to4 =
      model_extract_->ComputeLpcsOrientation(geometric_constants.scanner_mount_angle, delta_rot_y, delta_rot_z);
  // const Matrix3d R_4to10 = R_3to4.transpose();                                               // Rotation VTCS rel
  // LPCS

  // Torch clock position (when at toch plane ref point)
  const double clock_pos_torch =
      clock_pos_functor_.ComputeTorchPlaneClockPosition(R_10to11, R_3to4, clock_pos_laser, a_4to10);
  const Vector3d t_3to4 =
      clock_pos_functor_.ComputeTcsToLpcsTranslation(R_10to11, R_3to4, clock_pos_laser, clock_pos_torch, a_4to10);

  const Vector3d rot_center_1 = clock_pos_functor_.ComputeRotationCenter(R_10to11, clock_pos_laser, clock_pos_torch);
  // The basis of the following calculation is that the touch sense point in MCS (g_1) is physically
  // the same point as the fitted ABW circles intersection with MCS plane (f_11).
  // This means we have a common point expressed in two CS with known relative rotation which
  // can be used to solve for the translation vector between the systems. Exactly like in standard LTC.

  const Vector3d rotax_11 = {1.0, 0.0, 0.0};
  const Vector3d rotax_10 = R_10to11 * rotax_11;

  CalibrationResult result{};
  result.torch_to_lpcs_translation.c1 = t_3to4(0);
  result.torch_to_lpcs_translation.c2 = t_3to4(1);
  result.torch_to_lpcs_translation.c3 = t_3to4(2);
  result.rotation_center.c1           = rot_center_1(0);
  result.rotation_center.c2           = rot_center_1(1);
  result.rotation_center.c3           = rot_center_1(2);
  result.weld_object_rotation_axis.c1 = rotax_10(0);
  result.weld_object_rotation_axis.c2 = rotax_10(1);
  result.weld_object_rotation_axis.c3 = rotax_10(2);
  result.delta_rot_y                  = delta_rot_y;
  result.delta_rot_z                  = delta_rot_z;

  // Fill the CalibrationResult with some goodness of fit metrics.
  ComputeModelQuality(result, observations, geometric_constants, torch_plane_info.top_center_at_torch_plane);

  return result;
}

auto CalibrationSolverImpl::ComputeModelQuality(CalibrationResult &result, const std::vector<Observation> &observations,
                                                const GeometricConstants &constants,
                                                const macs::Point ref_point_macs) const -> void {
  lpcs::Point point_lpcs;
  macs::Point point_mcs;
  macs::Point residual;
  macs::Point mean_pos{.horizontal = 0.0, .vertical = 0.0};

  double max_residual{0.0};
  double residual_sum_of_squares{0.0};
  double total_sum_of_squares{0.0};
  int sample_size = observations.size();

  // Degrees of freedom
  int residual_df = sample_size - NBR_MODEL_PARAMS - 1;
  int total_df    = sample_size - 1;

  int count = 1;

  for (const auto &obs : observations) {
    point_lpcs = {.x = (obs.abw_points_lpcs.at(TOP_LEFT_ABW_IND).x + obs.abw_points_lpcs.at(TOP_RIGHT_ABW_IND).x) / 2,
                  .y = (obs.abw_points_lpcs.at(TOP_LEFT_ABW_IND).y + obs.abw_points_lpcs.at(TOP_RIGHT_ABW_IND).y) / 2};

    point_mcs = this->model_extract_->TransformAndRotateToTorchPlane(
        result.rotation_center, {constants.scanner_mount_angle, result.delta_rot_y, result.delta_rot_z},
        result.weld_object_rotation_axis, result.torch_to_lpcs_translation, point_lpcs, obs.slide_position);
    result.projected_points.push_back(point_mcs);

    residual                 = point_mcs - ref_point_macs;
    residual_sum_of_squares += residual.SquaredNorm();
    mean_pos                 = (mean_pos * (count - 1) + obs.slide_position) / count;
    max_residual             = std::max(max_residual, residual.Norm());
    count++;
  }

  for (const auto &proj_point_mcs : result.projected_points) {
    total_sum_of_squares += (proj_point_mcs - mean_pos).SquaredNorm();
  }

  result.max_residual            = max_residual;
  result.standard_deviation      = std::sqrt(total_sum_of_squares / sample_size);
  result.torch_plane_reference   = ref_point_macs;
  result.residual_sum_of_squares = residual_sum_of_squares;
  result.r_squared               = 1 - residual_sum_of_squares / total_sum_of_squares;
  result.adjusted_r_squared      = 1 - (1 - result.r_squared) * total_df / residual_df;
  result.residual_standard_error = std::sqrt(residual_sum_of_squares / (sample_size - 2));
}

auto CalibrationSolverImpl::SolveClockPosition(const GeometricConstants &geometric_constants,
                                               const TorchPlaneInfo &torch_plane_info,
                                               const std::vector<Observation> &observations)
    -> std::pair<bool, Eigen::VectorXd> {
  const Eigen::Vector2d center_slide_pos = {torch_plane_info.top_center_at_torch_plane.horizontal,
                                            torch_plane_info.top_center_at_torch_plane.vertical};

  Eigen::Matrix3d R_1to4;
  Eigen::Matrix3d R_10to4;

  // Setup functor for optimization
  clock_pos_functor_.model_extract    = this->model_extract_;
  clock_pos_functor_.observations     = observations;      //"move around" measurements
  clock_pos_functor_.torch_plane_info = torch_plane_info;  // Touch sense measurments
  // clock_pos_functor_.R_10to4              = R_1to4;
  clock_pos_functor_.scanner_mount_angle  = geometric_constants.scanner_mount_angle;
  clock_pos_functor_.object_radius        = geometric_constants.object_radius;
  clock_pos_functor_.ltc_torch_plane_dist = geometric_constants.ltc_laser_plane_distance;
  clock_pos_functor_.ltc_stickout         = geometric_constants.ltc_stickout;

  // TODO(zachjz): Add loop around the following to iterate over new start values if first one fails.
  Eigen::VectorXd params;
  params = ComputeStartValues(observations, center_slide_pos, geometric_constants.object_radius);
  Eigen::NumericalDiff<ClockPosFunctor> num_diff_wrapped_functor(clock_pos_functor_);
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<ClockPosFunctor>, double> optimizer(num_diff_wrapped_functor);
  optimizer.parameters.maxfev = MAX_ITER;
  Status status               = optimizer.minimize(params);

  LOG_INFO("SolveClockPosition, status: {}, params: {}", static_cast<int>(status), ToString(params));

  if (status != Eigen::LevenbergMarquardtSpace::RelativeErrorTooSmall &&
      status != Eigen::LevenbergMarquardtSpace::RelativeReductionTooSmall &&
      status != Eigen::LevenbergMarquardtSpace::RelativeErrorAndReductionTooSmall) {
    // Unexpected -> could set new start values and continue to new attempt here.
    // Returning true and checking RSE where called
    LOG_INFO("Accepting unexpected calibration solver status: {}", static_cast<int>(status));

    return {true, params};
  }

  return {true, params};
}

auto CalibrationSolverImpl::ComputeStartValues(const std::vector<Observation> &observations,
                                               const Eigen::Vector2d &center_slide_pos, double object_radius)
    -> Eigen::VectorXd {
  Eigen::Vector2d current_pos{};
  Eigen::Vector2d current_diff{};
  double min_dist       = INFINITY;
  int closest_point_idx = 0;
  int count             = 0;

  // Find measured point closest to center
  for (auto const &obs : observations) {
    current_pos(0) = obs.slide_position.horizontal;
    current_pos(1) = obs.slide_position.vertical;
    current_diff   = current_pos - center_slide_pos;

    if (current_diff.norm() < min_dist) {
      min_dist          = current_diff.norm();
      closest_point_idx = count;
    }

    count++;
  }

  // Start clock pos for torch
  // const double init_torch_clock_pos = INIT_LASER_CLOCK_POS - (INIT_L2T_DIST_MM / object_radius);

  Eigen::VectorXd start_values(NBR_MODEL_PARAMS);
  start_values(LASER_CLOCK_PARAM_IDX) = INIT_LASER_CLOCK_POS;
  start_values(AX_PARAM_IDX)          = observations.at(closest_point_idx).abw_points_lpcs.at(PRIMARY_ABW).x;
  start_values(AY_PARAM_IDX)          = observations.at(closest_point_idx).abw_points_lpcs.at(PRIMARY_ABW).y;

  if (OPTIMIZE_ORIENTATIONS) {
    start_values(OBJECT_ROT_PARAM_IDX) = 0.0;
    start_values(DELTA_Y_PARAM_IDX)    = 0.0;
    start_values(DELTA_Z_PARAM_IDX)    = 0.0;
  }

  return start_values;
}

auto CalibrationSolverImpl::CreateRotationMatrix(double angle, Eigen::Vector3d &rot_axis) -> Eigen::Matrix3d {
  rot_axis.normalize();
  Eigen::Matrix3d transform;
  transform = Eigen::Matrix3d::Zero();

  double factor = 0.0;
  if (!std::isnan(angle)) {
    factor = 1.0 - std::cos(angle);

    // This can be initilized much easier in Eigen. Doing this way to ensure that definitions are exactly the same as
    // in Scilab simulator. This definition of the transformation is equivalent to T = Trans4D * Rotation4D.
    transform(0, 0) = factor * std::pow(rot_axis(0), 2) + std::cos(angle);
    transform(0, 1) = factor * rot_axis(0) * rot_axis(1) - rot_axis(2) * std::sin(angle);
    transform(0, 2) = factor * rot_axis(0) * rot_axis(2) + rot_axis(1) * std::sin(angle);
    transform(1, 0) = factor * rot_axis(0) * rot_axis(1) + rot_axis(2) * std::sin(angle);
    transform(1, 1) = factor * std::pow(rot_axis(1), 2) + std::cos(angle);
    transform(1, 2) = factor * rot_axis(1) * rot_axis(2) - rot_axis(0) * std::sin(angle);
    transform(2, 0) = factor * rot_axis(2) * rot_axis(0) - rot_axis(1) * std::sin(angle);
    transform(2, 1) = factor * rot_axis(2) * rot_axis(1) + rot_axis(0) * std::sin(angle);
    transform(2, 2) = factor * std::pow(rot_axis(2), 2) + std::cos(angle);
  } else {
    transform = Eigen::Matrix3d::Identity();
  }

  return transform;
}

//}  // Namespace calibration

// NOLINTEND(*readability-identifier-naming*)
