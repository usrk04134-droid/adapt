#pragma once

#include "src/torch-base.h"

namespace deposition_simulator {

// Declaration of interfaces for the supported torch types
// Add additonal ones as needed.
// Reason for having separate interfaces instead of just the common
// TorchBase is that interfaces are expected to diverge as more
// functionality is added.

const double DEFAULT_TIP_DIAM    = 0.010;
const double DEFAULT_BASE_DIAM   = 0.020;
const double DEFAULT_CONE_LENGTH = 0.050;
const double DEFAULT_BASE_LENGTH = 0.030;

class ISingleWireTorch  //: public TorchBase
{
 public:
  virtual ~ISingleWireTorch()                                                                          = 0;
  virtual auto SetWireFeedSpeed(double wire_feed_speed) -> void                                        = 0;
  virtual auto GetWireFeedSpeed() const -> double                                                      = 0;
  virtual auto SetTorchGeometry(double tip_diameter, double base_diameter, double cone_length) -> void = 0;
};

class ITwinTorch  //: public TorchBase
{
 public:
  virtual ~ITwinTorch()                                                                                = 0;
  virtual auto SetWireFeedSpeed(double wire_feed_speed) -> void                                        = 0;
  virtual auto GetWireFeedSpeed() const -> double                                                      = 0;
  virtual auto SetTorchGeometry(double tip_diameter, double base_diameter, double cone_length) -> void = 0;
};

}  // Namespace deposition_simulator
