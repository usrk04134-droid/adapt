#pragma once

#include <Eigen/Eigen>

#include "object-positioner.h"
#include "src/coordinate-transformer.h"
#include "src/cwo.h"

namespace deposition_simulator {

class RollerChuck : public ObjectPositioner {
 private:
  std::shared_ptr<CircularWeldObject> weld_object_;
  std::shared_ptr<CoordinateTransformer> transformer_;
  double total_drift_{0.0};
  // Eigen::Vector3d trans_opcs_from_macs_; //M_1->9

 public:
  RollerChuck(std::shared_ptr<CircularWeldObject> &weld_object, std::shared_ptr<CoordinateTransformer> &transformer);
  ~RollerChuck() override = default;
  auto PositionObject(double desired_slice_angle_at_torch, double drift_increment) -> void override;
  auto SetWeldObject(std::shared_ptr<CircularWeldObject> &cwo) -> void override;
  auto SetCoordinateTransformer(std::shared_ptr<CoordinateTransformer> &transformer) -> void override;
  auto GetTotalDrift() const -> double override;
};

}  // Namespace deposition_simulator
