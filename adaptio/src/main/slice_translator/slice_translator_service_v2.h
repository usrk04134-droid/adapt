#pragma once

#include <optional>
#include <vector>

#include "common/groove/point.h"
#include "lpcs/lpcs_point.h"

namespace slice_translator {

class SliceTranslatorServiceV2 {
 public:
  virtual ~SliceTranslatorServiceV2() = default;

  virtual auto LPCSToMCS(const std::vector<lpcs::Point>& lpcs_points, const common::Point& slide_position) const
      -> std::optional<std::vector<common::Point>> = 0;
  virtual auto MCSToLPCS(const std::vector<common::Point>& macs_points, const common::Point& slide_position) const
      -> std::optional<std::vector<lpcs::Point>>                                                             = 0;
  virtual auto DistanceFromTorchToScanner(const std::vector<lpcs::Point>& lpcs_points,
                                          const common::Point& axis_position) const -> std::optional<double> = 0;
  virtual auto Available() const -> bool                                                                     = 0;
};

}  // namespace slice_translator
