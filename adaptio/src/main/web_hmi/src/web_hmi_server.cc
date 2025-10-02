#include "web_hmi_server.h"

#include <crow/app.h>
#include <crow/http_request.h>
#include <crow/http_response.h>

#include <cstdint>
#include <filesystem>
#include <fstream>

#include "common/clock_functions.h"
#include "common/file/yaml.h"
#include "common/logging/application_log.h"
#include "common/logging/component_logger.h"
#include "common/time/format.h"
#include "image_logging/image_logging_manager.h"
#include "json_payload.h"
#include "slice_translator/slice_translator.h"
#include "web_hmi/handlers/bead_control.h"
#include "web_hmi/handlers/kinematics.h"
#include "web_hmi/handlers/lane_control.h"
#include "web_hmi/handlers/scanner.h"
#include "web_hmi/handlers/settings.h"
#include "web_hmi/handlers/weld_sequence.h"

namespace web_hmi {

WebHmiServer::WebHmiServer(controller::OperationControl* operation_control, kinematics::KinematicsClient* kinematics_client,
                           scanner_client::ScannerClient* scanner_client, slice_translator::SliceTranslator* slice_translator,
                           image_logging::ImageLoggingManager* image_logging_manager, SQLite::Database* db,
                           clock_functions::SystemClockNowFunc system_clock_now_func, const std::filesystem::path& path_logs,
                           prometheus::Registry* registry,
                           const std::optional<std::filesystem::path>& path_last_scanner_images)
    : comps_{.operation_control        = operation_control,
             .kinematics_client        = kinematics_client,
             .scanner_client           = scanner_client,
             .slice_translator         = slice_translator,
             .image_logging_manager    = image_logging_manager},
      system_clock_now_func_(system_clock_now_func),
      logger_({.component = "webhmi", .path_directory = path_logs / "webhmi", .file_name = "%Y%m%d_%H%M%S.log"}),
      path_log_dir_(path_logs / "webhmi"),
      path_last_scanner_images_(path_last_scanner_images) {
  // Register external metrics
  (void)registry;
}

void WebHmiServer::Start() {
  SubscribeToWebSocketEvents();
  SubscribeToHttpEvents();

  app_.port(8080).multithreaded().run();
}

void WebHmiServer::Stop() { app_.stop(); }

void WebHmiServer::OnScannerStarted(bool success) { (void)success; }

void WebHmiServer::OnScannerStopped(bool success) { (void)success; }

void WebHmiServer::Receive(const macs::Slice& machine_data, const lpcs::Slice& scanner_data, const common::groove::Point& slides_actual,
                           double angle_from_torch_to_scanner) {
  cached_mcs_                      = machine_data;
  cached_lpcs_                     = scanner_data;
  cached_axis_position_            = {.horizontal = slides_actual.horizontal, .vertical = slides_actual.vertical};
  cached_angle_from_torch_to_scanner_ = angle_from_torch_to_scanner;
}

}  // namespace web_hmi
