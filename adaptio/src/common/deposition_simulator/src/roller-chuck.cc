
#include "src/roller-chuck.h"

#include <cmath>
#include <Eigen/Eigen>
#include <memory>

#include "point3d.h"
#include "src/coordinate-transformer.h"
#include "src/cwo.h"

namespace deposition_simulator {

RollerChuck::RollerChuck(std::shared_ptr<CircularWeldObject> &weld_object,
                         std::shared_ptr<CoordinateTransformer> &transformer)
    : weld_object_(weld_object), transformer_(transformer) {}

auto RollerChuck::PositionObject(double desired_slice_angle_at_torch, double drift_increment) -> void {
  Matrix4d transform;

  this->total_drift_ += drift_increment;

  if (std::isnan(desired_slice_angle_at_torch)) {
    // TODO(zachjz): Review this solution. Ideally there should be a way
    //  to check if the tranformer is ready for transforming. I.e. has
    //  been initialized with all relevant relations.
    transform = Eigen::Matrix4d::Identity(4, 4).eval();
  } else {
    const Point3d torch_pos_opcs    = transformer_->GetTorchPos(OPCS);
    const double torch_angle_opcs   = std::atan2(-torch_pos_opcs.GetY(), torch_pos_opcs.GetZ());
    const double required_rot_angle = desired_slice_angle_at_torch - torch_angle_opcs;
    Eigen::Vector3d rotax_rocs{1, 0, 0};  // Fixed for RollerChuck
    Eigen::Vector3d translation{total_drift_, 0, 0};
    // M_2->9
    transform = CoordinateTransformer::CreateTransform(rotax_rocs, required_rot_angle, translation, false);
  }

  this->transformer_->SetWeldObjectOrientation(transform);
}

auto RollerChuck::SetCoordinateTransformer(std::shared_ptr<CoordinateTransformer> &transformer) -> void {
  this->transformer_ = transformer;
}

auto RollerChuck::SetWeldObject(std::shared_ptr<CircularWeldObject> &cwo) -> void { this->weld_object_ = cwo; }

auto RollerChuck::GetTotalDrift() const -> double { return this->total_drift_; }
}  // Namespace deposition_simulator
