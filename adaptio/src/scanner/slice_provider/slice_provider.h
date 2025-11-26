#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include <boost/uuid/uuid.hpp>

#include "common/groove/groove.h"
#include "scanner/image/camera_model.h"
#include "scanner/image/image.h"
#include "scanner/image/image_types.h"
#include "scanner/joint_model/joint_model.h"

namespace scanner::slice_provider {

enum class SliceConfidence { NO, LOW, MEDIUM, HIGH };

using Timestamp = std::chrono::time_point<std::chrono::high_resolution_clock>;

struct JointSlice {
  std::optional<image::RawImageData> image_data;
  boost::uuids::uuid uuid;
  Timestamp timestamp;
  std::string image_name;
  scanner::joint_model::JointProfile profile;
  image::WorkspaceCoordinates centroids;

  uint64_t num_walls_found = 0;
  uint64_t processing_time;
  int vertical_crop_start;
  bool approximation_used;
  std::array<common::Point, joint_model::INTERPOLATED_SNAKE_SIZE> snake_points;
};

class SliceProvider {
 public:
  virtual ~SliceProvider()                                        = default;
  virtual void AddSlice(const JointSlice& slice)                  = 0;
  virtual auto GetSlice() -> std::optional<joint_model::JointProfile> = 0;
  virtual auto GetTrackingSlice()
      -> std::optional<std::tuple<common::Groove, std::array<common::Point, joint_model::INTERPOLATED_SNAKE_SIZE>,
                                  SliceConfidence, uint64_t>> = 0;
  virtual auto SliceDegraded() -> bool                        = 0;
  virtual void Reset()                                        = 0;
};

using SliceProviderPtr = std::unique_ptr<SliceProvider>;

}  // namespace scanner::slice_provider
