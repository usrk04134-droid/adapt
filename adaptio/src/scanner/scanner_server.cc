#include "scanner_server.h"

#include <array>
#include <cstdint>
#include <Eigen/Eigen>
#include <optional>

#include "common/logging/application_log.h"
#include "common/messages/scanner.h"
#include "common/zevs/zevs_socket.h"
#include "scanner/joint_tracking/joint_slice.h"

namespace scanner {

const double MM_PER_METER = 1000.0;

ScannerServer::ScannerServer(zevs::SocketPtr socket) : socket_(socket) { LOG_DEBUG("Creating ScannerServer"); }

void ScannerServer::ScannerOutput(const joint_tracking::JointSlice& joint_slice, const std::optional<double> area,
                                  uint64_t time_stamp, joint_tracking::SliceConfidence confidence) {
  common::msg::scanner::SliceData input{
      .groove_area = area.has_value() ? area.value() : 0.,
  };

  // Redo this when JointSlice is updated
  for (int i = 0; i < common::msg::scanner::GROOVE_ARRAY_SIZE; i++) {
    auto x_mm       = MM_PER_METER * joint_slice.GetElement(i).x;
    auto z_mm       = MM_PER_METER * joint_slice.GetElement(i).z;
    input.groove[i] = {x_mm, z_mm};
  }

  switch (confidence) {
    case joint_tracking::SliceConfidence::HIGH:
      input.confidence = common::msg::scanner::SliceConfidence::HIGH;
      break;
    case joint_tracking::SliceConfidence::MEDIUM:
      input.confidence = common::msg::scanner::SliceConfidence::MEDIUM;
      break;
    case joint_tracking::SliceConfidence::LOW:
      input.confidence = common::msg::scanner::SliceConfidence::LOW;
      break;
    case joint_tracking::SliceConfidence::NO:
      input.confidence = common::msg::scanner::SliceConfidence::NO;
      break;
  }

  input.time_stamp = time_stamp;
  socket_->Send(input);
}

}  // namespace scanner
