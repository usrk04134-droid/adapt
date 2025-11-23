#pragma once
#include "common/groove/groove.h"
#include "common/zevs/zevs_socket.h"
#include "scanner/core/src/scanner.h"

namespace scanner {

class ScannerServer : public ScannerOutputCB {
 public:
  explicit ScannerServer(zevs::SocketPtr socket);

  ScannerServer(ScannerServer&)                     = delete;
  auto operator=(ScannerServer&) -> ScannerServer&  = delete;
  ScannerServer(ScannerServer&&)                    = delete;
  auto operator=(ScannerServer&&) -> ScannerServer& = delete;

  virtual ~ScannerServer() = default;

  void ScannerOutput(const common::Groove& groove,
                     const std::array<common::Point, joint_model::INTERPOLATED_SNAKE_SIZE>& profile,
                     uint64_t time_stamp, slice_provider::SliceConfidence confidence) override;

 private:
  zevs::SocketPtr socket_;
};

using ScannerServerPtr = std::unique_ptr<ScannerServer>;

}  // namespace scanner
