#include "core/scanner/image_provider/basler_camera.h"

#include <core/logging/application_log.h>
#include <GenApi/INodeMap.h>
#include <prometheus/gauge.h>
#include <prometheus/registry.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/Container.h>
#include <pylon/ETimeoutHandling.h>
#include <pylon/FeaturePersistence.h>
#include <pylon/GrabResultPtr.h>
#include <pylon/PylonBase.h>
#include <pylon/TlFactory.h>
#include <pylon/TypeMappings.h>

#include <boost/outcome/result.hpp>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <system_error>
#include <utility>

#include "core/image/image.h"
#include "core/image/image_builder.h"
#include "core/image/image_types.h"
#include "core/scanner/scanner_types.h"

using core::image::Image;
using core::scanner::BaslerCamera;
using core::scanner::BaslerCameraErrorCode;
using core::scanner::BaslerCameraUpstreamImageEventHandler;

using namespace GenApi;
using namespace Pylon;

namespace {
std::string const TEMPERATURE_STATUS_OK       = "ok";
std::string const TEMPERATURE_STATUS_CRITICAL = "critical";
std::string const TEMPERATURE_STATUS_ERROR    = "error";
auto const TEMPERATURE_STATUSES = {TEMPERATURE_STATUS_OK, TEMPERATURE_STATUS_CRITICAL, TEMPERATURE_STATUS_ERROR};
auto const GET_SCANNER_METRICS_INTERVAL = std::chrono::seconds(15);
}  // namespace

const char* pfsFileContent = R"(
@PFS_FILE_CONTENT@
)";

BaslerCameraUpstreamImageEventHandler::BaslerCameraUpstreamImageEventHandler(ImageProvider::OnImage on_image,
                                                                             Timestamp start_time, int64_t start_tick,
                                                                             int original_vertical_offset,
                                                                             int original_horizontal_offset)
    : base_timestamp(start_time),
      base_tick(start_tick),
      original_vertical_offset_(original_vertical_offset),
      original_horizontal_offset_(original_horizontal_offset),
      on_image_(std::move(on_image)) {}

void BaslerCameraUpstreamImageEventHandler::OnImageGrabbed(CInstantCamera& camera, const CGrabResultPtr& grab_result) {
  if (!(grab_result.IsValid())) {
    LOG_ERROR("Invalid grab result");
    return;
  }

  if (grab_result->GrabSucceeded()) {
    auto* buffer = static_cast<uint8_t*>(grab_result->GetBuffer());

    auto height   = grab_result->GetHeight();
    auto width    = grab_result->GetWidth();
    auto offset_y = grab_result->GetOffsetY();
    auto offset_x = grab_result->GetOffsetX();

    const auto delay = std::chrono::milliseconds(5 + 75 * height / 2500);

    auto image_data = core::image::RawImageData(Eigen::Map<core::image::RawImageData>(buffer, height, width));
    auto image      =
        core::image::ImageBuilder::From(image_data, offset_y - original_vertical_offset_,
                                        offset_x - original_horizontal_offset_)
            .Finalize()
            .value();

    image->SetTimestamp(std::chrono::high_resolution_clock::now() - delay);

    on_image_(std::move(image));
  } else {
    std::string s(grab_result->GetErrorDescription());
    LOG_ERROR("Grab did not succeed: code {}, description {}.", grab_result->GetErrorCode(), s);
  }
}

void BaslerCameraUpstreamImageEventHandler::OnImagesSkipped(CInstantCamera& camera, size_t countOfSkippedImages) {
  LOG_TRACE("{} images from the camera were skipped.", countOfSkippedImages);
}

BaslerCamera::BaslerCamera(const BaslerConfig& config, const Fov& fov, prometheus::Registry* registry)
    : config_(config), fov_(fov) {
  SetupMetrics(registry);
}

BaslerCamera::~BaslerCamera() = default;

auto BaslerCamera::Start(enum ScannerSensitivity sensitivity) -> boost::outcome_v2::result<void> {
  if (on_image_ == nullptr) {
    LOG_ERROR("Starting the camera but the image handler has not been set.");
    return BOOST_OUTCOME_V2_NAMESPACE::failure(BaslerCameraErrorCode::IMAGE_EVENT_HANDLER_NOT_SET);
  }

  LOG_TRACE("Start grabbing");
  if (!Started()) {
    float gain          = config_.gain;
    float exposure_time = config_.exposure_time;
    int steps_to_increase;
    switch (sensitivity) {
      case core::scanner::ScannerSensitivity::NORMAL:
        steps_to_increase = 0;
        break;
      case core::scanner::ScannerSensitivity::HIGH:
        steps_to_increase = 1;
        break;
    }

    while (steps_to_increase > 0 && gain <= 18.0) {
      gain += 6.0;
      steps_to_increase--;
    }
    while (steps_to_increase > 0) {
      exposure_time *= 2.0;
      steps_to_increase--;
    }

    initial_gain_ = gain;

    auto result = InitializeCamera(gain, exposure_time);
    if (!result) {
      return result;
    }
    LOG_INFO("Initialized camera");
    camera_->GetStreamGrabberParams().DestinationPort = 50100;

    INodeMap& nodemap = camera_->GetNodeMap();
    // Take a "snapshot" of the camera's current timestamp value
    CCommandParameter(nodemap, "TimestampLatch").Execute();
    auto tp = std::chrono::high_resolution_clock::now();
    // Get the timestamp value
    int64_t i = CIntegerParameter(nodemap, "TimestampLatchValue").GetValue();

    auto on_image = [this](std::unique_ptr<Image> image) {
      on_image_(std::move(image));

      auto const now = std::chrono::steady_clock::now();
      if (now > last_get_scanner_metrics_ + GET_SCANNER_METRICS_INTERVAL) {
        UpdateMetrics();
        last_get_scanner_metrics_ = now;
      }
    };

    auto upstream_handler =
        new BaslerCameraUpstreamImageEventHandler(on_image, tp, i, fov_.offset_y, fov_.offset_x);
    camera_->RegisterImageEventHandler(upstream_handler, ERegistrationMode::RegistrationMode_ReplaceAll,
                                       ECleanup::Cleanup_Delete);

    // Start grabbing
    camera_->StartGrabbing(EGrabStrategy::GrabStrategy_LatestImageOnly, EGrabLoop::GrabLoop_ProvidedByInstantCamera);
    LOG_INFO("Continuous grabbing started.");
  }
  return BOOST_OUTCOME_V2_NAMESPACE::success();
}

void BaslerCamera::ResetFOVAndGain() {
  if (!camera_) {
    return;
  }
  camera_->StopGrabbing();
  camera_->OffsetY.SetValue(0);
  camera_->Height.SetValue(fov_.height);
  camera_->OffsetY.SetValue(fov_.offset_y);
  camera_->Gain.SetValue(initial_gain_);
  camera_->StartGrabbing(EGrabStrategy::GrabStrategy_LatestImageOnly, EGrabLoop::GrabLoop_ProvidedByInstantCamera);
}

void BaslerCamera::SetVerticalFOV(int offset_from_top, int height) {
  if (!camera_) {
    return;
  }
  camera_->StopGrabbing();
  camera_->OffsetY.SetValue(0);
  if (fov_.offset_y + offset_from_top + height > fov_.height) {
    height = fov_.height - offset_from_top - fov_.offset_y;
  }
  camera_->Height.SetValue(height);
  camera_->OffsetY.SetValue(fov_.offset_y + offset_from_top);
  camera_->StartGrabbing(EGrabStrategy::GrabStrategy_LatestImageOnly, EGrabLoop::GrabLoop_ProvidedByInstantCamera);
  LOG_TRACE("Continuous grabbing restarted with offset {} and height {}.", fov_.offset_y + offset_from_top, height);
};

void BaslerCamera::AdjustGain(double factor) {
  if (!camera_) {
    return;
  }
  camera_->StopGrabbing();
  auto gain     = camera_->Gain.GetValue();
  auto new_gain = gain + log10(factor) / log10(2) * 6.0;
  if (new_gain < 0.0) {
    new_gain = 0.0;
  } else if (new_gain > 24.0) {
    new_gain = 24.0;
  }
  camera_->Gain.SetValue(new_gain);
  camera_->StartGrabbing(EGrabStrategy::GrabStrategy_LatestImageOnly, EGrabLoop::GrabLoop_ProvidedByInstantCamera);
  LOG_TRACE("Continuous grabbing restarted with gain {:.3f} dB * {:.3f} = {:.3f} dB", gain, factor, new_gain);
};

auto BaslerCamera::GetVerticalFOVOffset() -> int { return camera_->OffsetY.GetValue() - fov_.offset_y; };

auto BaslerCamera::GetVerticalFOVHeight() -> int { return camera_->Height.GetValue(); };

void BaslerCamera::Stop() {
  if (Started()) {
    camera_->StopGrabbing();
    camera_->Close();
    camera_.reset();
    PylonTerminate();
  }
  UpdateMetrics();
}

auto BaslerCamera::Started() const -> bool { return camera_ && camera_->IsOpen(); }

auto BaslerCamera::StartCamera() -> boost::outcome_v2::result<void> {
  auto& factory = CTlFactory::GetInstance();

  auto num_retries_remaining = 6;
  auto num_devices           = 0;
  DeviceInfoList_t devices{};

  while (num_retries_remaining >= 0) {
    devices     = {};
    num_devices = factory.EnumerateDevices(devices);
    if (num_devices == 0) {
      if (num_retries_remaining == 0) {
        LOG_ERROR("Camera detection failed");
        return BaslerCameraErrorCode::NO_CAMERA_FOUND;
      }

      boost::this_thread::sleep(boost::posix_time::milliseconds(300));
    } else {
      break;
    }

    num_retries_remaining--;
    LOG_INFO("Retrying camera detection, num_retries_remaining={}", num_retries_remaining);
  }

  num_retries_remaining = 6;

  LOG_INFO("Found {}/{} devices", num_devices, devices.size());

  for (auto& device : devices) {
    LOG_INFO("{}: {}", device.GetDeviceID().c_str(), device.GetAddress().c_str());
  }

  while (!Started() && num_retries_remaining >= 0) {
    try {
      camera_ =
          std::make_unique<Pylon::CBaslerUniversalInstantCamera>(Pylon::CTlFactory::GetInstance().CreateFirstDevice());
    } catch (const Pylon::GenericException& e) {
      if (num_retries_remaining == 0) {
        LOG_ERROR("CreateFirstDevice failed");
        return BaslerCameraErrorCode::FAILED_TO_ACQUIRE_CAMERA;
      }
      num_retries_remaining--;
      LOG_INFO("Retrying createFirstDevice, num_retries_remaining={}", num_retries_remaining);
      boost::this_thread::sleep(boost::posix_time::milliseconds(300));
      continue;
    }

    try {
      camera_->Open();
    } catch (const Pylon::GenericException& e) {
      if (num_retries_remaining == 0) {
        LOG_ERROR("Camera open failed");
        return BaslerCameraErrorCode::FAILED_TO_OPEN_CAMERA;
      }
    }

    if (Started()) {
      break;
    }

    num_retries_remaining--;
    boost::this_thread::sleep(boost::posix_time::milliseconds(300));
    LOG_INFO("Retrying Camera open, num_retries_remaining={}", num_retries_remaining);
  }

  return BOOST_OUTCOME_V2_NAMESPACE::success();
}

auto BaslerCamera::GetSerialNumber() -> std::string {
  auto const already_started = Started();

  LOG_TRACE("camera already started: {}", already_started ? "yes" : "no");

  auto ok = true;
  std::string serial_number;

  if (!already_started) {
    PylonInitialize();

    auto const res = StartCamera();
    if (!res) {
      LOG_ERROR("Start camera failed!");
      ok = false;
    }
  }

  if (ok) {
    serial_number = std::string(camera_->GetDeviceInfo().GetSerialNumber().c_str());
  }

  if (!already_started) {
    Stop();
  }

  return serial_number;
}

auto BaslerCamera::InitializeCamera(float gain, float exposure_time) -> boost::outcome_v2::result<void> {
  PylonInitialize();

  auto const res = StartCamera();
  if (!res) {
    PylonTerminate();
    return res;
  }

  LOG_INFO("{}", std::string("Using Device ").append(camera_->GetDeviceInfo().GetProductId()));
  LOG_INFO("Camera Firmware Version: {}", std::string(camera_->DeviceFirmwareVersion()));
  LOG_INFO("Camera Serial number: {}", camera_->GetDeviceInfo().GetSerialNumber().c_str());

  try {
    INodeMap& node_map = camera_->GetNodeMap();
    CFeaturePersistence::LoadFromString(pfsFileContent, &node_map, true);
    camera_->Width.SetValue(fov_.width);
    camera_->Height.SetValue(fov_.height);
    camera_->OffsetX.SetValue(fov_.offset_x);
    camera_->OffsetY.SetValue(fov_.offset_y);
    camera_->ExposureTime.SetValue(exposure_time);
    camera_->Gain.SetValue(gain);
    return BOOST_OUTCOME_V2_NAMESPACE::success();
  } catch (const Pylon::GenericException& e) {
    LOG_ERROR("Pylon error occurred: {}", e.GetDescription());
  } catch (...) {
    LOG_ERROR("An unknown error occured.");
  }

  PylonTerminate();

  return BaslerCameraErrorCode::FAILED_TO_SET_CONFIG;
}

void BaslerCamera::SetupMetrics(prometheus::Registry* registry) {
  {
    auto& gauges = prometheus::BuildGauge()
                       .Name("basler_camera_temperate_status")
                       .Help("Basler camera's current temperature status.")
                       .Register(*registry);
    metrics_.temperature_status_gauges.emplace("ok", gauges.Add({
                                                         {"status", "ok"}
    }));
    metrics_.temperature_status_gauges.emplace("critical", gauges.Add({
                                                               {"status", "critical"}
    }));
    metrics_.temperature_status_gauges.emplace("error", gauges.Add({
                                                            {"status", "error"}
    }));
  }

  {
    auto& gauge = prometheus::BuildGauge()
                      .Name("basler_camera_status_error_count")
                      .Help("Basler camera's total status error count.")
                      .Register(*registry);
    metrics_.temperature_status_error = &gauge.Add({});
  }

  {
    auto& gauge = prometheus::BuildGauge()
                      .Name("basler_camera_temperature")
                      .Help("Basler camera's current temperature (in celcius).")
                      .Register(*registry);
    metrics_.temperature = &gauge.Add({});
  }

  {
    auto& gauge = prometheus::BuildGauge()
                      .Name("basler_camera_max_temperature")
                      .Help("Basler camera's maximum temperature (in celcius).")
                      .Register(*registry);
    metrics_.max_temperature = &gauge.Add({});
  }

  UpdateMetrics();
}

void BaslerCamera::UpdateMetrics() {
  auto temperature_status             = TEMPERATURE_STATUS_OK;
  auto temperature_status_error_count = 0.0;
  auto temperature                    = 0.0;
  auto max_temperature                = 0.0;

  if (camera_) {
    auto const temp_status = camera_->BslTemperatureStatus.GetValue();
    switch (temp_status) {
      case Basler_UniversalCameraParams::BslTemperatureStatus_Critical:
        temperature_status = TEMPERATURE_STATUS_CRITICAL;
        break;
      case Basler_UniversalCameraParams::BslTemperatureStatus_Error:
        temperature_status = TEMPERATURE_STATUS_ERROR;
        break;
      case Basler_UniversalCameraParams::BslTemperatureStatus_Ok:
        break;
    }

    temperature_status_error_count = static_cast<double>(camera_->BslTemperatureStatusErrorCount.GetValue());
    max_temperature                = camera_->BslTemperatureMax.GetValue();
    temperature                    = camera_->DeviceTemperature.GetValue();
  }

  for (const auto& status : TEMPERATURE_STATUSES) {
    metrics_.temperature_status_gauges.at(status).Set(status == temperature_status ? 1 : 0);
  }

  metrics_.temperature_status_error->Set(temperature_status_error_count);
  metrics_.temperature->Set(temperature);
  metrics_.max_temperature->Set(max_temperature);
}

// Error code implementation
namespace {

struct ErrorCategory : std::error_category {
  auto name() const noexcept -> const char* final;          // NOLINT(*-use-nodiscard)
  auto message(int error_code) const -> std::string final;  // NOLINT(*-use-nodiscard)
  auto default_error_condition(int other) const noexcept    // NOLINT(*-use-nodiscard)
      -> std::error_condition final;                        // NOLINT(*-use-nodiscard)
};

auto ErrorCategory::name() const noexcept -> const char* { return "BaslerCameraError"; }

auto ErrorCategory::message(int error_code) const -> std::string {
  switch (static_cast<BaslerCameraErrorCode>(error_code)) {
    case BaslerCameraErrorCode::NO_ERROR:
      return "No error";
    case BaslerCameraErrorCode::NO_CAMERA_FOUND:
      return "No camera found";
    case BaslerCameraErrorCode::FAILED_TO_ACQUIRE_CAMERA:
      return "Failed to aquire camera";
    case BaslerCameraErrorCode::FAILED_TO_SET_CONFIG:
      return "Failed to set configuration";
    case BaslerCameraErrorCode::FAILED_TO_OPEN_CAMERA:
      return "Failed to open camera";
    case BaslerCameraErrorCode::IMAGE_EVENT_HANDLER_NOT_SET:
      return "Image event handler was not set";
  }
}

auto ErrorCategory::default_error_condition(int other) const noexcept -> std::error_condition {
  switch (static_cast<BaslerCameraErrorCode>(other)) {
    default:
      return {other, *this};
  }
}

const ErrorCategory ERROR_CATEGORY{};

}  // namespace

[[maybe_unused]] auto core::scanner::make_error_code(BaslerCameraErrorCode error_code)
    -> std::error_code {  // NOLINT(*-identifier-naming)
  return {static_cast<int>(error_code), ERROR_CATEGORY};
}
