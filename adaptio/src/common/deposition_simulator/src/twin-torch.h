#pragma once

#include "../torch-interfaces.h"

namespace deposition_simulator {

class TwinTorch : public ITwinTorch, public TorchBase {
 private:
  double wire_feed_speed_{};
  double wire_diameter_{};
  double tip_diameter_{DEFAULT_TIP_DIAM};
  double base_diameter_{DEFAULT_BASE_DIAM};
  double cone_length_{DEFAULT_CONE_LENGTH};

 public:
  TwinTorch(double wire_diam, double initial_wire_feed_speed);
  ~TwinTorch() override = default;
  auto GetVolumeDepositionRate() const -> double override;
  auto SetWireFeedSpeed(double wire_feed_speed) -> void override;
  auto GetWireFeedSpeed() const -> double override;
  auto GetWireDiameter() const -> double override;
  auto SetTorchGeometry(double tip_diameter, double base_diameter, double cone_length) -> void override;
  auto GetVertices() const -> std::vector<Point3d> override;
};

}  // Namespace deposition_simulator
