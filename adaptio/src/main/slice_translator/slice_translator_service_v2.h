#pragma once

#include <optional>
#include <vector>

#include "lpcs/lpcs_point.h"
#include "macs/macs_point.h"

namespace slice_translator {

class SliceTranslatorServiceV2 {
 public:
  virtual ~SliceTranslatorServiceV2() = default;

  virtual auto LPCSToMCS(const std::vector<lpcs::Point>& lpcs_points, const macs::Point& slide_position)
      -> std::optional<std::vector<macs::Point>> = 0;
  virtual auto MCSToLPCS(const std::vector<macs::Point>& macs_points, const macs::Point& slide_position)
      -> std::optional<std::vector<lpcs::Point>> = 0;
  virtual auto AngleFromTorchToScanner(const std::vector<lpcs::Point>& lpcs_points, const macs::Point& axis_position)
      -> std::optional<double>           = 0;
  virtual auto Available() const -> bool = 0;
};

}  // namespace slice_translator
