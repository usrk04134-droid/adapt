#pragma once

#include <cstdint>
#include <vector>

#include "common/messages/scanner.h"
#include "common/zevs/zevs_socket.h"
#include "joint_geometry/joint_geometry.h"
#include "kinematics/kinematics_client.h"
#include "scanner_client/scanner_client.h"

namespace scanner_client {

class ScannerClientImpl : public ScannerClient {
 public:
  explicit ScannerClientImpl(zevs::Socket* socket, kinematics::KinematicsClient* kinematics_client,
                             double joint_tolorance_upper_width, double joint_tolerance_surface_angle,
                             double joint_tolerance_wall_angle);

  ScannerClientImpl(ScannerClientImpl&)                     = delete;
  auto operator=(ScannerClientImpl&) -> ScannerClientImpl&  = delete;
  ScannerClientImpl(ScannerClientImpl&&)                    = delete;
  auto operator=(ScannerClientImpl&&) -> ScannerClientImpl& = delete;

  ~ScannerClientImpl() override = default;

  void Start(const Config& config, const joint_geometry::JointGeometry& joint_geometry) override;
  void Update(const UpdateData& data) override;
  void Stop() override;

  void AddObserver(ScannerObserver* observer) override;
  void ImageLoggingUpdate(const ImageLoggingData& data) override;
  void FlushImageBuffer() override;

 private:
  void OnSliceData(common::msg::scanner::SliceData slice_data);
  void OnStartRsp(common::msg::scanner::StartRsp started);
  void OnStopRsp(common::msg::scanner::StopRsp started);
  void OnGetSlidesPosition(std::uint64_t time_stamp, double horizontal, double vertical);
  void UpdateImageLogger(const ImageLoggingData& data);

  std::vector<ScannerObserver*> observers_;
  zevs::Socket* socket_;
  kinematics::KinematicsClient* kinematics_client_;
  bool started_;
  lpcs::Slice scanner_data_in_process_ = {};
  double joint_tolerance_upper_width_{};
  double joint_tolerance_surface_angle_{};
  double joint_tolerance_wall_angle_{};
  common::msg::scanner::JointGeometry joint_geometry_{};
};

}  // namespace scanner_client
