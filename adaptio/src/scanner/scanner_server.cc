#include "scanner_server.h"

#include <array>
#include <cstdint>
#include <Eigen/Eigen>
#include <optional>

#include "common/groove/groove.h"
#include "common/logging/application_log.h"
#include "common/messages/scanner.h"
#include "common/zevs/zevs_socket.h"

namespace scanner {

const double MM_PER_METER = 1000.0;

ScannerServer::ScannerServer(zevs::SocketPtr socket) : socket_(socket) { LOG_DEBUG("Creating ScannerServer"); }

void ScannerServer::ScannerOutput(const common::Groove& groove, const std::vector<common::Point>& line,
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

  for (std::size_t i = 0; i < common::msg::scanner::LINE_ARRAY_SIZE; i++) {
    if (i < line.size()) {
      input.line[i] = {.x = MM_PER_METER * line[i].horizontal, .y = MM_PER_METER * line[i].vertical};
    } else {
      input.line[i] = {.x = 0.0, .y = 0.0};
    }
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

  input.time_stamp = time_stamp;
  socket_->Send(input);
}

}  // namespace scanner
