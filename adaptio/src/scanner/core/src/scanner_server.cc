#include "scanner/core/src/scanner_server.h"

#include <cstdint>
#include <Eigen/Eigen>

#include "common/groove/groove.h"
#include "common/logging/application_log.h"
#include "common/messages/scanner.h"
#include "common/zevs/zevs_socket.h"
#include "scanner/slice_provider/slice_provider.h"

namespace scanner {

const double MM_PER_METER = 1000.0;

ScannerServer::ScannerServer(zevs::SocketPtr socket) : socket_(socket) { LOG_DEBUG("Creating ScannerServer"); }

void ScannerServer::ScannerOutput(const common::Groove& groove, const std::array<common::Point, 100>& profile,
                                  uint64_t time_stamp, slice_provider::SliceConfidence confidence) {
  common::msg::scanner::SliceData input{
      .groove_area = groove.Area(),
  };

  // Redo this when JointSlice is updated
  for (int i = 0; i < common::msg::scanner::GROOVE_ARRAY_SIZE; i++) {
    auto x_mm       = MM_PER_METER * groove[i].horizontal;
    auto z_mm       = MM_PER_METER * groove[i].vertical;
    input.groove[i] = {.x = x_mm, .y = z_mm};
  }

  switch (confidence) {
    case scanner::slice_provider::SliceConfidence::HIGH:
      input.confidence = common::msg::scanner::SliceConfidence::HIGH;
      break;
    case scanner::slice_provider::SliceConfidence::MEDIUM:
      input.confidence = common::msg::scanner::SliceConfidence::MEDIUM;
      break;
    case scanner::slice_provider::SliceConfidence::LOW:
      input.confidence = common::msg::scanner::SliceConfidence::LOW;
      break;
    case scanner::slice_provider::SliceConfidence::NO:
      input.confidence = common::msg::scanner::SliceConfidence::NO;
      break;
  }

  for (int i = 0; i < common::msg::scanner::PROFILE_ARRAY_SIZE; i++) {
    auto x_mm        = MM_PER_METER * profile[i].horizontal;
    auto z_mm        = MM_PER_METER * profile[i].vertical;
    input.profile[i] = {x_mm, z_mm};
  }
  input.time_stamp = time_stamp;
  socket_->Send(input);
}

}  // namespace scanner
