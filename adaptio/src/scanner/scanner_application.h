#pragma once

#include <prometheus/registry.h>

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include "common/messages/scanner.h"
#include "common/zevs/zevs_core.h"
#include "common/zevs/zevs_socket.h"
#include "scanner/image_logger/image_logger.h"
#include "scanner/image_provider/image_provider.h"
#include "scanner/image_provider/image_provider_configuration.h"
#include "scanner/joint_model/joint_model.h"
#include "scanner/scanner.h"
#include "scanner/scanner_calibration_configuration.h"
#include "scanner/scanner_configuration.h"
#include "scanner/scanner_server.h"

namespace scanner {

class ScannerApplication {
 public:
  explicit ScannerApplication(const ScannerConfigurationData& scanner_config,
                              const ScannerCalibrationData& scanner_calib, const image_provider::Fov& fov,
                              image_provider::ImageProvider* image_provider, const std::string& endpoint_base_url,
                              const std::optional<std::filesystem::path>& data_path,
                              image_logger::ImageLogger* image_logger, prometheus::Registry* registry);

  ScannerApplication(ScannerApplication&)                     = delete;
  auto operator=(ScannerApplication&) -> ScannerApplication&  = delete;
  ScannerApplication(ScannerApplication&&)                    = delete;
  auto operator=(ScannerApplication&&) -> ScannerApplication& = delete;

  virtual ~ScannerApplication() = default;

  void ThreadEntry(const std::string& name);
  void StartThread(const std::string& event_loop_name);
  void JoinThread();
  void Exit();

 private:
  auto GetCoreScanner(const joint_model::JointProperties& properties) -> ScannerPtr;

  void Start(common::msg::scanner::Start data);
  void Stop(common::msg::scanner::Stop data);
  void Update(common::msg::scanner::Update data);
  void ImageLoggingUpdate(common::msg::scanner::ImageLoggingData data);
  void FlushImageBuffer(common::msg::scanner::FlushImageBuffer flush);
  void OnTimeout();
  auto static MsgDataToJointProperties(common::msg::scanner::JointGeometry joint_geometry)
      -> joint_model::JointProperties;
  std::thread thread_;
  zevs::SocketPtr socket_;
  ScannerPtr core_scanner_;
  zevs::TimerPtr timer_;
  zevs::EventLoopPtr event_loop_;
  std::optional<uint32_t> timer_task_id_;
  ScannerConfigurationData scanner_config_;
  ScannerCalibrationData scanner_calib_;
  image_provider::ImageProvider* image_provider_;
  std::string scanner_endpoint_url_;
  image_logger::ImageLogger* image_logger_;
  std::optional<std::filesystem::path> data_path_;
  ScannerServerPtr scanner_server_;
  image_provider::Fov fov_;
  prometheus::Registry* registry_;
};

using ScannerApplicationPtr = std::unique_ptr<ScannerApplication>;

}  // namespace scanner
