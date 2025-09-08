#pragma once

#include <Eigen/Eigen>

#include "src/coordinate-transformer.h"
#include "src/cwo.h"

namespace deposition_simulator {

class ObjectPositioner {
 public:
  virtual ~ObjectPositioner()                                                                        = 0;
  virtual auto SetWeldObject(std::shared_ptr<CircularWeldObject> &cwo) -> void                       = 0;
  virtual auto SetCoordinateTransformer(std::shared_ptr<CoordinateTransformer> &transformer) -> void = 0;
  virtual auto GetTotalDrift() const -> double                                                       = 0;

  // Positions the weld object by imposing contraints on where the torch
  // is positioned and additional contraints from the postioner implementation.
  // Results from positioning are used to update the coordinate transformer.
  virtual auto PositionObject(double desired_slice_angle_at_torch, double drift_increment)
      -> void = 0;  // Rotation around ROCS x (not the current rotax)
};

}  // Namespace deposition_simulator
