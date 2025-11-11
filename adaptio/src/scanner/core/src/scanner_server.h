#pragma once
#include "common/groove/groove.h"
#include "common/zevs/zevs_socket.h"
#include "scanner/core/scanner.h"

namespace scanner {

class ScannerServer : public ScannerOutputCB {
 public:
  explicit ScannerServer(zevs::SocketPtr socket);

  ScannerServer(ScannerServer&)                     = delete;
  auto operator=(ScannerServer&) -> ScannerServer&  = delete;
  ScannerServer(ScannerServer&&)                    = delete;
  auto operator=(ScannerServer&&) -> ScannerServer& = delete;

  virtual ~ScannerServer() = default;

  void ScannerOutput(const common::Groove& groove, const std::array<common::Point, 100>& profile, uint64_t time_stamp,
                     slice_provider::SliceConfidence confidence) override;

 private:
  zevs::SocketPtr socket_;
};

using ScannerServerPtr = std::unique_ptr<ScannerServer>;

}  // namespace scanner
