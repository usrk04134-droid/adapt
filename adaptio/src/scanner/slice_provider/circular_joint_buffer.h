#pragma once

#include <boost/circular_buffer.hpp>
#include <chrono>
#include <memory>
#include <optional>
#include <vector>

#include "scanner/image/camera_model.h"
#include "scanner/image/image.h"
#include "scanner/image/image_types.h"
#include "scanner/joint_model/joint_model.h"

namespace scanner::slice_provider {

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

class CircularJointBuffer {
 public:
  CircularJointBuffer();

  void AddSlice(const JointSlice& slice);

  [[nodiscard]] auto GetSlice() const -> std::optional<JointSlice>;

  [[nodiscard]] auto GetLatestTimestamp() const -> std::optional<Timestamp>;

  [[nodiscard]] auto GetRecentSlices(long time_period_ms) const -> std::vector<JointSlice*>;

  [[nodiscard]] auto GetNumberOfSlices() const -> uint64_t;

  void Reset();

 private:
  boost::circular_buffer<JointSlice> m_buffer;
};

}  // namespace scanner::slice_provider
