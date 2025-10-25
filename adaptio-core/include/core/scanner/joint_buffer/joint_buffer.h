#pragma once

#include <memory>
#include <optional>

#include "core/image/camera_model.h"
#include "core/image/image.h"
#include "core/image/image_types.h"
#include "core/scanner/joint_model.h"

namespace core::scanner {

using Timestamp = std::chrono::time_point<std::chrono::high_resolution_clock>;

struct JointSlice {
  std::optional<core::image::RawImageData> image_data;
  boost::uuids::uuid uuid;
  Timestamp timestamp;
  std::string image_name;
  core::scanner::JointProfile profile;
  core::image::WorkspaceCoordinates centroids;
  uint64_t num_walls_found = 0;
  uint64_t processing_time;
  int vertical_crop_start;
  int horizontal_crop_start_ = 0;
};

class JointBuffer {
 public:
  virtual ~JointBuffer()                                               = default;
  virtual void AddSlice(const JointSlice& slice)                       = 0;
  virtual auto GetSlice() const -> std::optional<JointSlice>           = 0;
  virtual auto GetLatestTimestamp() const -> std::optional<Timestamp>  = 0;
  virtual auto GetRecentSlices(long) const -> std::vector<JointSlice*> = 0;
  virtual auto GetNumberOfSlices() const -> uint64_t                   = 0;
  virtual void Reset()                                                 = 0;
};

using JointBufferPtr = std::unique_ptr<JointBuffer>;

}  // namespace core::scanner
