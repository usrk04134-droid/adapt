#pragma once
#include "common/zevs/zevs_socket.h"
#include "scanner/joint_tracking/joint_slice.h"
#include "scanner/scanner.h"

namespace scanner {

class ScannerServer : public ScannerOutputCB {
 public:
  explicit ScannerServer(zevs::SocketPtr socket);

  ScannerServer(ScannerServer&)                     = delete;
  auto operator=(ScannerServer&) -> ScannerServer&  = delete;
  ScannerServer(ScannerServer&&)                    = delete;
  auto operator=(ScannerServer&&) -> ScannerServer& = delete;

  virtual ~ScannerServer() = default;

  void ScannerOutput(const joint_tracking::JointSlice& joint_slice, const std::optional<double> area, uint64_t time_stamp,
                     joint_tracking::SliceConfidence confidence) override;

 private:
  zevs::SocketPtr socket_;
};

using ScannerServerPtr = std::unique_ptr<ScannerServer>;

}  // namespace scanner
