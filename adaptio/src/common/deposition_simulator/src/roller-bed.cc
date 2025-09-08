#include "roller-bed.h"

#include <unsupported/Eigen/src/NonLinearOptimization/HybridNonLinearSolver.h>
#include <unsupported/Eigen/src/NonLinearOptimization/LevenbergMarquardt.h>

#include <cmath>
#include <Eigen/Eigen>
#include <memory>
#include <stdexcept>
#include <unsupported/Eigen/NonLinearOptimization>

#include "point3d.h"
#include "src/coordinate-transformer.h"
#include "src/cwo.h"

namespace deposition_simulator {

using Eigen::HybridNonLinearSolverSpace::Status;

// const Eigen::Vector<double, 6> DEFAULT_PARAMS({0.1, 0.0, 0.0, 0.0, 0.0, -1.0});
const double MIN_ANG_DIFF           = 1e-5;  // Min angle to consider in dot product. If smaller, then treat as zer0
const int MAX_TRIALS                = 10;
const double TRIAL_STEP_EULER1      = 0.02;   // Default step on euler 1 when trying new start
const double EULER1_START           = 0.1;    // Start value of euler angle 1
const double TRANS_Z_START_FRACTION = 0.9;    // Fraction of radius to use as start val
const int MAX_FEV                   = 10000;  // Max nbr of calls to cost function
const double XTOL                   = 1e-6;   // Tolerance required of optimization
const int NBR_SUB_DIAG              = 15;     // How many elements of the Jabobian that should be used
const int NBR_SUP_DIAG              = 15;     // How many elements of the Jabobian that should be used
const int END_STOP_IND              = 4;
const int TORCH_IND                 = 5;
const int EULER_X_IND               = 0;
const int EULER_Y_IND               = 1;
const int EULER_Z_IND               = 2;
const int TRANS_X_IND               = 3;
const int TRANS_Y_IND               = 4;
const int TRANS_Z_IND               = 5;

RollerBedFunctor::RollerBedFunctor(std::shared_ptr<CircularWeldObject> &cwo) : cwo(cwo) {}

auto RollerBedFunctor::operator()(const Eigen::VectorXd &parameters, Eigen::VectorXd &fvec) const -> int {
  Eigen::Matrix4d opcs_to_rocs;  // M_2->9
  opcs_to_rocs = RollerBedFunctor::CreateTransformationMatrix(parameters);

  Eigen::MatrixXd contact_points_rocs = opcs_to_rocs * contact_points_opcs;

  double polar_angle      = NAN;
  double contact_distance = NAN;
  double surface_distance = NAN;
  // double torch_distance = NAN;

  // Handle fvec(0...3), i.e. P1-P4 (roller wheel contact points)
  for (int i = 0; i < NBR_CONTACT_POINTS - 2; i++) {
    polar_angle      = std::atan2(-contact_points_rocs(1, i), contact_points_rocs(2, i));  //[ROCSY, ROCSZ]
    contact_distance = std::sqrt(std::pow(contact_points_rocs(1, i), 2) + std::pow(contact_points_rocs(2, i), 2));
    surface_distance = cwo->GetSurfaceDistance(contact_points_rocs(0, i), polar_angle);
    fvec(i)          = surface_distance - contact_distance;  // Error in radial dir.
  }

  // Handle fvec(4) i.e. P5 (endstop)
  const double x_max = cwo->GetMaxX();  // TODO(zachjz): Change this to GetMaxXvalAtPolarAngle(polar_angle)
  fvec(END_STOP_IND) = x_max - contact_points_rocs(0, END_STOP_IND);  // Error in x_rocs dir

  // Handle fvec(5) i.e. P6 (torch plane)
  //  //Target: the actual position of the torch
  Eigen::Vector3d target_torch_vector = {0.0, contact_points_rocs(1, TORCH_IND), contact_points_rocs(2, TORCH_IND)};
  // //Current: radial vector in rocs plane to be moved to torch location. Nope...
  Eigen::Vector3d actual_torch_vector = {0.0, -std::sin(torch_plane_angle_rocs), std::cos(torch_plane_angle_rocs)};

  // torch_distance = target.norm();
  target_torch_vector.normalize();
  actual_torch_vector.normalize();

  const double dot_prod = target_torch_vector.dot(actual_torch_vector);
  // double target_angle = atan2(-contact_points_rocs(1,5), contact_points_rocs(2,5));

  Vector3d cross_prod  = target_torch_vector.cross(actual_torch_vector);
  double angle_diff    = dot_prod > 1.0 ? 0.0 : std::acos(dot_prod);
  const int angle_sign = cross_prod(0) < 0.0 ? -1.0 : 1.0;

  if (std::abs(angle_diff) > MIN_ANG_DIFF) {
    angle_diff = angle_diff * angle_sign;
  } else {
    angle_diff = 0.0;
  }

  // fvec(5) = torch_distance * angle_diff; //Error, arc length to torch plane
  fvec(TORCH_IND) = angle_diff;
  // std::cout << std::setprecision(8) << std::fixed;
  // std::cout << "-------------------------------\n";
  // std::cout << "Target angle:     " << target_angle << "\n";
  // std::cout << "Torch plane angle:" << torch_plane_angle_rocs << "\n";
  // std::cout << "Dot prod:         " << dot_prod << "\n";
  // std::cout << "Ang diff:         " << angle_diff << "\n";
  // std::cout << "Cross prod:       " << cross_prod << "\n";
  // std::cout << "fvec:             " << fvec.transpose() << "\n";
  // std::cout << "params:           " << parameters.transpose() << "\n";

  // if (std::isnan(angle_diff)){
  //   std::cout << opcs_to_rocs << "\n";
  // }

  return 0;
}

auto RollerBedFunctor::CreateTransformationMatrix(const Eigen::Vector<double, NBR_PARAMETERS> &parameters)
    -> Eigen::Matrix4d {
  Eigen::Matrix4d transform;  // M_2->9

  // Optimization parameters, the translation and orientation of ROCS
  // Vector3d rotax = parameters.block(0, 0, 3, 1);
  // double rotang = rotax.norm();
  // rotax.normalize();

  const double euler1 = parameters(EULER_X_IND);
  const double euler2 = parameters(EULER_Y_IND);
  const double euler3 = parameters(EULER_Z_IND);
  const double transx = parameters(TRANS_X_IND);
  const double transy = parameters(TRANS_Y_IND);
  const double transz = parameters(TRANS_Z_IND);

  Eigen::Matrix3d rotmat;
  rotmat = Eigen::AngleAxis<double>(euler1, Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxis<double>(euler2, Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxis<double>(euler3, Eigen::Vector3d::UnitZ());

  // rotmat = Eigen::AngleAxis<double>(rotang, rotax).toRotationMatrix();

  // if (helpers::pmod(rotang, 2 * std::numbers::pi) < 1e-5 || helpers::pmod(rotang, 2 * std::numbers::pi) >
  // 2*std::numbers::pi - 1e-5) {
  //   std::cout << "rotang: " << rotang << ", " << helpers::pmod(rotang, 2 * std::numbers::pi) << "\n";
  //   rotmat = Eigen::Matrix3d::Identity();
  // }
  // else {
  // rotax = -rotax;
  // rotang = -rotang;
  // rotmat = Eigen::AngleAxis<double>(rotang, rotax).toRotationMatrix();
  //}

  transform.block(0, 0, 3, 3) = rotmat;
  transform(0, 3)             = transx;
  transform(1, 3)             = transy;
  transform(2, 3)             = transz;
  transform(3, 3)             = 1.0;

  return transform;
}

RollerBed::RollerBed(std::shared_ptr<CircularWeldObject> &weld_object,
                     std::shared_ptr<CoordinateTransformer> &transformer)
    : weld_object_(weld_object), transformer_(transformer) {}

auto RollerBed::PositionObject(double desired_slice_angle_at_torch, double drift_increment) -> void {
  // TODO(zachjz): Here we shall solve system of non-linear equations that give
  // us the 5 contact points (4 roller + 1 side) between roller bed and weld object
  // Shall  update the transformer with the orientation of ROCS relative to MACS
  // std::ofstream opcs_file;
  // opcs_file.open("opcs_file.txt", std::ios_base::app);
  this->roller_offset_opcs_x_ += drift_increment;

  // Check that object center plane, i.e. the joint, has not moved outside the roller bed
  Point3d object_center_rocs = {0, 0, 0, ROCS};
  Point3d object_center_opcs = this->transformer_->Transform(object_center_rocs, OPCS);

  if (object_center_opcs.GetX() > roller_sep_opcs_x_ / 2 || object_center_opcs.GetX() < -roller_sep_opcs_x_ / 2) {
    throw std::runtime_error(
        "Joint is not allowed to drift outside roller-bed wheels. Reduce drift or increase width of rollerbed in opcs "
        "x");
  }

  const Point3d torchpos_opcs = transformer_->GetTorchPos(OPCS);
  const double sep_y          = this->roller_sep_opcs_y_;
  const double sep_x          = this->roller_sep_opcs_x_;
  const double off_x          = this->roller_offset_opcs_x_;

  // 0-3: roller wheel contact points, 4: endstop, 5: torchpos
  Eigen::MatrixXd rollerbed_points_opcs(4, TORCH_IND + 1);
  rollerbed_points_opcs << sep_x / 2, -sep_x / 2, -sep_x / 2, sep_x / 2, off_x, torchpos_opcs.GetX(), -sep_y / 2,
      -sep_y / 2, sep_y / 2, sep_y / 2, 0, torchpos_opcs.GetY(), 0, 0, 0, 0, 0, torchpos_opcs.GetZ(), 1, 1, 1, 1, 1, 1;

  // Call HybridNonLinearSolver() to get rotation axis and angle and translations.
  RollerBedFunctor rb_functor(weld_object_);
  rb_functor.contact_points_opcs = rollerbed_points_opcs;
  rb_functor.torch_plane_angle_rocs =
      desired_slice_angle_at_torch;  // helpers::pmod(desired_slice_angle_at_torch, 2 * std::numbers::pi);
  Eigen::HybridNonLinearSolver<RollerBedFunctor> hybrid_solver(rb_functor);
  Eigen::VectorXd params(this->model_parameters_);

  hybrid_solver.parameters.maxfev               = MAX_FEV;
  hybrid_solver.parameters.xtol                 = XTOL;
  hybrid_solver.parameters.nb_of_subdiagonals   = NBR_SUB_DIAG;
  hybrid_solver.parameters.nb_of_superdiagonals = NBR_SUP_DIAG;

  bool success                = false;
  int trial_count             = 0;
  double euler1_start         = this->model_parameters_(1);
  const Vector3d rotvec_start = this->model_parameters_.block(0, 0, 3, 1);
  // double rotang = rotvec_start.norm();
  while (trial_count < MAX_TRIALS) {
    const Status opt_status = hybrid_solver.solveNumericalDiff(params);

    if (opt_status == Eigen::HybridNonLinearSolverSpace::RelativeErrorTooSmall) {
      success = true;
      break;
    }

    // If progress is poor, try new start point. Primarily try to change main
    // rotation around x.
    if (opt_status != Eigen::HybridNonLinearSolverSpace::RelativeErrorTooSmall) {
      euler1_start -= TRIAL_STEP_EULER1;
      params        = Eigen::VectorXd(this->model_parameters_);
      params(1)     = euler1_start;
      // params.block(0,0,3,1) = rotvec_start.normalized() * (rotang - 0.02 * trial_count);
      //  if (trial_count % 2 == 0) {
      //    params.block(0,0,3,1) = rotvec_start.normalized() * (rotang - 0.05 * (trial_count / 2 + 1));
      //  }
      //  else{
      //    params.block(0,0,3,1) = rotvec_start.normalized() * (rotang + 0.05 * ((trial_count - 1) / 2 + 1));
      //  }
    } else {
      break;
    }

    trial_count++;
  }

  if (!success) {
    throw std::runtime_error(
        "Failed to position weld object. Optimization does not converge. Is rollerbed config reasonable?");
  }

  this->model_parameters_ = params;
  Eigen::Matrix4d opcs_rel_to_rocs;  // M_2->9
  opcs_rel_to_rocs = RollerBedFunctor::CreateTransformationMatrix(this->model_parameters_);
  this->transformer_->SetWeldObjectOrientation(opcs_rel_to_rocs);

  // opcs_file << "matrix: " << opcs_rel_to_rocs.reshaped(1, 16) << "\n";
  // opcs_file << "params: " << this->model_parameters_.transpose() << "\n";
  // opcs_file << "slice_at_torch: " << desired_slice_angle_at_torch << "\n";
  // opcs_file << "trial_count: " << trial_count << "\n";
  // opcs_file.close();
}

// NOLINTNEXTLINE(*easily-swappable*)
auto RollerBed::SetRollerBedGeometry(double axle_sep, double wheel_sep, double drift_stop_offset) -> void {
  this->roller_sep_opcs_x_    = wheel_sep;
  this->roller_sep_opcs_y_    = axle_sep;
  this->roller_offset_opcs_x_ = drift_stop_offset;
  this->model_parameters_     = {EULER1_START, 0.0, 0.0,
                                 0.0,          0.0, -this->weld_object_->GetMaxRadius() * TRANS_Z_START_FRACTION};  // 2->9
}

auto RollerBed::SetCoordinateTransformer(std::shared_ptr<CoordinateTransformer> &transformer) -> void {
  this->transformer_ = transformer;
}

auto RollerBed::SetWeldObject(std::shared_ptr<CircularWeldObject> &cwo) -> void { this->weld_object_ = cwo; }

auto RollerBed::GetTotalDrift() const -> double {
  // TODO(zachjz): Here we should return the "drift" at a certain position of the end of the object
  // Basically simulating what the edge sensor would read at a particular point. If the object
  // wobbles, the sensor reading will be different depending on position.
  return 0.0;
}
}  // Namespace deposition_simulator
