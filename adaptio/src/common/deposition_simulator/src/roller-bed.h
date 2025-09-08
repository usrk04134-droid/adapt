#pragma once

#include <Eigen/Eigen>
#include <unsupported/Eigen/NonLinearOptimization>

#include "coordinate-transformer.h"
#include "object-positioner.h"
#include "src/cwo.h"

namespace deposition_simulator {

const int NBR_CONTACT_POINTS = 6;
const int NBR_PARAMETERS     = 6;

struct RollerBedFunctor {
  double torch_plane_angle_rocs{0.0};
  Eigen::Matrix<double, 4, NBR_CONTACT_POINTS> contact_points_opcs;
  std::shared_ptr<CircularWeldObject> cwo;

  explicit RollerBedFunctor(std::shared_ptr<CircularWeldObject> &cwo);

  auto operator()(const Eigen::VectorXd &parameters, Eigen::VectorXd &fvec) const -> int;

  static auto CreateTransformationMatrix(const Eigen::Vector<double, NBR_PARAMETERS> &parameters) -> Eigen::Matrix4d;
};

class RollerBed : public ObjectPositioner {
 private:
  std::shared_ptr<CircularWeldObject> weld_object_;
  std::shared_ptr<CoordinateTransformer> transformer_;
  // Roller positions are defined w.r.t to the static CLCS
  double roller_sep_opcs_x_{};
  double roller_sep_opcs_y_{};
  double roller_offset_opcs_x_{};  // How much center line of roller bed is offset from CLCS in x-dir.
  Eigen::Vector<double, NBR_PARAMETERS> model_parameters_;

 public:
  RollerBed(std::shared_ptr<CircularWeldObject> &weld_object, std::shared_ptr<CoordinateTransformer> &transformer);
  ~RollerBed() override = default;
  auto SetRollerBedGeometry(double axle_sep, double wheel_sep, double drift_stop_offset) -> void;
  auto PositionObject(double desired_slice_angle_at_torch, double drift_increment) -> void override;
  auto SetWeldObject(std::shared_ptr<CircularWeldObject> &cwo) -> void override;
  auto SetCoordinateTransformer(std::shared_ptr<CoordinateTransformer> &transformer) -> void override;
  auto GetTotalDrift() const -> double override;
};

}  // Namespace deposition_simulator
