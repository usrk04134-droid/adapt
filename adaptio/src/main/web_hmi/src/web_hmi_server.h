#pragma once

#include <crow/json.h>
#include <prometheus/registry.h>
#include <SQLiteCpp/Database.h>

#include <functional>
#include <memory>
#include <optional>

#include "common/clock_functions.h"
#include "common/json_helpers.h"
#include "common/logging/application_log.h"
#include "common/time/format.h"
#include "controller/operation_control.h"
#include "event_handler/event_handler.h"
#include "image_logging/image_logging_manager.h"
#include "joint_geometry/joint_geometry.h"
#include "kinematics/kinematics_client.h"
#include "lpcs/lpcs_slice.h"
#include "macs/macs_slice.h"
#include "scanner_client/scanner_client.h"
#include "slice_translator/slice_translator.h"
#include "web_hmi.h"

namespace web_hmi {

class WebHmiServer : public WebHmi, public scanner_client::ScannerObserver, public slice_translator::SliceObserver {
 public:
  WebHmiServer(controller::OperationControl* operation_control, kinematics::KinematicsClient* kinematics_client,
               scanner_client::ScannerClient* scanner_client, slice_translator::SliceTranslator* slice_translator,
               image_logging::ImageLoggingManager* image_logging_manager, SQLite::Database* db,
               clock_functions::SystemClockNowFunc system_clock_now_func, const std::filesystem::path& path_logs,
               prometheus::Registry* registry,
               const std::optional<std::filesystem::path>& path_last_scanner_images);

  void Start() override;
  void Stop() override;

  void OnScannerStarted(bool success) override;
  void OnScannerStopped(bool success) override;
  void OnScannerDataUpdate(const lpcs::Slice & /*data*/, const macs::Point & /*axis_position*/) override {};
  void Receive(const macs::Slice& machine_data, const lpcs::Slice& scanner_data, const common::groove::Point& slides_actual,
               double angle_from_torch_to_scanner) override;

 private:
  struct SessionConfig {
    std::string last_settings;
    std::string last_status;
    std::string last_weld_control_status;
  };

  struct Components {
    controller::OperationControl* operation_control;
    kinematics::KinematicsClient* kinematics_client;
    scanner_client::ScannerClient* scanner_client;
    slice_translator::SliceTranslator* slice_translator;
    image_logging::ImageLoggingManager* image_logging_manager;
  } comps_;

  crow::SimpleApp app_;
  clock_functions::SystemClockNowFunc system_clock_now_func_;
  common::logging::ComponentLogger logger_;

  std::optional<macs::Slice> cached_mcs_;
  std::optional<lpcs::Slice> cached_lpcs_;
  std::optional<macs::Point> cached_axis_position_;
  std::optional<double> cached_angle_from_torch_to_scanner_;
  std::optional<double> cached_weld_axis_scale_;

  std::filesystem::path path_log_dir_;
  const std::optional<std::filesystem::path> path_last_scanner_images_;

  void SubscribeToWebSocketEvents();
  void SubscribeToHttpEvents();

  void LaneControl();
  void SetBeadControlConfig();
  void SetSystemSettings();
  void WeldingSequence();
  void Scanner();
  void Kinematics();
  void MachineState();
  void ImageUpload();
  void ImageLogging();
  void Reset();
  void LastScannerImages();
};

}  // namespace web_hmi
