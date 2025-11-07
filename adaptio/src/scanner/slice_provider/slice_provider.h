#pragma once

#include <memory>
#include <optional>
#include <vector>

#include "common/groove/groove.h"
#include "common/groove/point.h"
#include "scanner/image/camera_model.h"
#include "scanner/image/image.h"
#include "scanner/image/image_types.h"
#include "scanner/joint_buffer/joint_buffer.h"
#include "scanner/joint_model/joint_model.h"

namespace scanner::slice_provider {

enum class SliceConfidence { NO, LOW, MEDIUM, HIGH };

using Timestamp = std::chrono::time_point<std::chrono::high_resolution_clock>;

class SliceProvider {
 public:
  virtual ~SliceProvider()                                                                                = default;
  virtual void AddSlice(const scanner::joint_buffer::JointSlice& slice)                                   = 0;
  virtual auto GetSlice() -> std::optional<joint_model::JointProfile>                                     = 0;
  virtual auto GetTrackingSlice()
      -> std::optional<std::tuple<common::Groove, std::vector<common::Point>, SliceConfidence, uint64_t>> = 0;
  virtual auto SliceDegraded() -> bool                                                                    = 0;
  virtual void Reset()                                                                                    = 0;
};

using SliceProviderPtr = std::unique_ptr<SliceProvider>;

}  // namespace scanner::slice_provider
