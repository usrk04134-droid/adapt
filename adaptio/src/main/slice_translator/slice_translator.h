#pragma once

#include <optional>

#include "kinematics/kinematics_client.h"
#include "lpcs/lpcs_slice.h"
#include "macs/macs_slice.h"
#include "slice_observer.h"

namespace slice_translator {

class SliceTranslator {
 public:
  virtual ~SliceTranslator() = default;

  virtual void AddObserver(SliceObserver* observer) = 0;
  virtual auto LPCSToMCS(const std::vector<lpcs::Point>& vec, const common::groove::Point& axis_position)
      -> std::optional<std::vector<macs::Point>> = 0;
  virtual auto LPCSToMCS(const lpcs::Groove& lpcs_groove, const common::groove::Point& axis_position)
      -> std::optional<std::vector<macs::Point>> = 0;
  virtual auto MCSToLPCS(const std::vector<macs::Point>& vec, const common::groove::Point& axis_position)
      -> std::optional<std::vector<lpcs::Point>> = 0;
  virtual auto AngleFromTorchToScanner(const lpcs::Groove& lpcs_groove, const common::groove::Point& axis_position)
      -> std::optional<double> = 0;
};

}  // namespace slice_translator
