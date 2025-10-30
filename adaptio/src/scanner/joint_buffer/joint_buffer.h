#pragma once

#include <memory>
#include <optional>

#include "scanner/image/camera_model.h"
#include "scanner/image/image.h"
#include "scanner/image/image_types.h"
#include "scanner/joint_model/joint_model.h"

namespace scanner::joint_buffer {

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
  int vertical_crop_start = 0;
  int horizontal_crop_start = 0;
  bool approximation_used;
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

}  // namespace scanner::joint_buffer
