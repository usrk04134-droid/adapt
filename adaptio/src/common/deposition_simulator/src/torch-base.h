#pragma once

#include <vector>

#include "point3d.h"
#include "src/line3d.h"
namespace deposition_simulator {

class TorchBase {
 public:
  virtual ~TorchBase()                                     = 0;
  virtual auto GetVolumeDepositionRate() const -> double   = 0;
  virtual auto GetWireDiameter() const -> double           = 0;
  virtual auto GetVertices() const -> std::vector<Point3d> = 0;
};

}  // namespace deposition_simulator
