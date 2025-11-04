#include "scanner/image_provider/basler_camera.h"

#include <GenApi/INodeMap.h>
#include <prometheus/gauge.h>
#include <prometheus/registry.h>
#include <pylon/_BaslerUniversalCameraParams.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/Container.h>
#include <pylon/ECleanup.h>
#include <pylon/ERegistrationMode.h>
#include <pylon/FeaturePersistence.h>
#include <pylon/GrabResultPtr.h>
#include <pylon/InstantCamera.h>
#include <pylon/PylonBase.h>
#include <pylon/TlFactory.h>
#include <pylon/TypeMappings.h>

#include <algorithm>
#include <boost/outcome/result.hpp>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <system_error>
#include <utility>

#include "common/logging/application_log.h"
#include "scanner/image/image.h"
#include "scanner/image/image_builder.h"
#include "scanner/image/image_types.h"
#include "scanner/image_provider/image_provider.h"
#include "scanner/image_provider/image_provider_configuration.h"
#include "scanner/scanner_types.h"

using namespace GenApi;
using namespace Pylon;

namespace {
std::string const TEMPERATURE_STATUS_OK       = "ok";
std::string const TEMPERATURE_STATUS_CRITICAL = "critical";
std::string const TEMPERATURE_STATUS_ERROR    = "error";
auto const TEMPERATURE_STATUSES = {TEMPERATURE_STATUS_OK, TEMPERATURE_STATUS_CRITICAL, TEMPERATURE_STATUS_ERROR};
auto const GET_SCANNER_METRICS_INTERVAL = std::chrono::seconds(15);
}  // namespace

namespace scanner::image_provider {

const double MAX_GAIN  = 24.0;
const double MIN_GAIN  = 0.0;
const double GAIN_STEP = 2.0;

const char* pfsFileContent = R"(
@PFS_FILE_CONTENT@
)";

BaslerCameraUpstreamImageEventHandler::BaslerCameraUpstreamImageEventHandler(ImageProvider::OnImage on_image,
                                                                             Timestamp start_time, int64_t start_tick,
                                                                             int original_offset,
                                                                             int original_offset_x)
    : base_timestamp(start_time),
      base_tick(start_tick),
      original_offset_(original_offset),
      original_offset_x_(original_offset_x),
      on_image_(std::move(on_image)) {}

void BaslerCameraUpstreamImageEventHandler::OnImageGrabbed(CInstantCamera& camera, const CGrabResultPtr& grab_result) {
  if (!(grab_result.IsValid())) {
    LOG_ERROR("Invalid grab result");
    return;
  }

  if (grab_result->GrabSucceeded()) {
    auto* buffer = static_cast<uint8_t*>(grab_result->GetBuffer());

    auto height = grab_result->GetHeight();
    auto width  = grab_result->GetWidth();
    auto offset = grab_result->GetOffsetY();

    const auto delay = std::chrono::milliseconds(5 + 75 * height / 2500);

    auto image_data = scanner::image::RawImageData(Eigen::Map<scanner::image::RawImageData>(buffer, height, width));
    auto horizontal_offset = original_offset_x_;
    auto image             =
        scanner::image::ImageBuilder::From(image_data, offset - original_offset_, horizontal_offset).Finalize().value();

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

auto BaslerCamera::Start(enum scanner::ScannerSensitivity sensitivity) -> boost::outcome_v2::result<void> {
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
      case scanner::ScannerSensitivity::NORMAL:
        steps_to_increase = 0;
        break;
      case scanner::ScannerSensitivity::HIGH:
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

    auto on_image = [this](std::unique_ptr<image::Image> image) {
      on_image_(std::move(image));

      auto const now = std::chrono::steady_clock::now();
      if (now > last_get_scanner_metrics_ + GET_SCANNER_METRICS_INTERVAL) {
        UpdateMetrics();
        last_get_scanner_metrics_ = now;
      }
    };

      auto upstream_handler =
          new BaslerCameraUpstreamImageEventHandler(on_image, tp, i, static_cast<int>(fov_.offset_y),
                                                    static_cast<int>(fov_.offset_x));
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

void BaslerCamera::SetHorizontalFOV(int offset_from_left, int width) {
  if (!camera_) {
    return;
  }

  camera_->StopGrabbing();
  camera_->OffsetX.SetValue(0);

  if (fov_.offset_x + offset_from_left + width > fov_.width) {
    width = fov_.width - offset_from_left - fov_.offset_x;
  }

  width = std::max(width, 1);

  camera_->Width.SetValue(width);
  camera_->OffsetX.SetValue(fov_.offset_x + offset_from_left);
  camera_->StartGrabbing(EGrabStrategy::GrabStrategy_LatestImageOnly, EGrabLoop::GrabLoop_ProvidedByInstantCamera);
  LOG_TRACE("Continuous grabbing restarted with horizontal offset {} and width {}.",
            fov_.offset_x + offset_from_left, width);
};

void BaslerCamera::AdjustGain(double factor) {
  if (!camera_) {
    return;
  }

  if (factor == 0.) {
    return;
  }

  auto gain = camera_->Gain.GetValue();
  auto step = log10(factor);

  if ((step < 0. && gain == MIN_GAIN) || (step > 0. && gain == MAX_GAIN)) {
    return;
  }

  camera_->StopGrabbing();

  auto new_gain = gain;
  if (step > 0.) {
    new_gain += GAIN_STEP;
  } else {
    new_gain -= GAIN_STEP;
  }

  new_gain = std::clamp(new_gain, MIN_GAIN, MAX_GAIN);

  camera_->Gain.SetValue(new_gain);
  camera_->StartGrabbing(EGrabStrategy::GrabStrategy_LatestImageOnly, EGrabLoop::GrabLoop_ProvidedByInstantCamera);
  LOG_TRACE("Continuous grabbing restarted with gain {:.3f} dB * {:.3f} = {:.3f} dB", gain, factor, new_gain);
};

auto BaslerCamera::GetVerticalFOVOffset() -> int { return camera_->OffsetY.GetValue() - fov_.offset_y; };

auto BaslerCamera::GetVerticalFOVHeight() -> int { return camera_->Height.GetValue(); };

auto BaslerCamera::GetHorizontalFOVOffset() -> int { return camera_->OffsetX.GetValue() - fov_.offset_x; };

auto BaslerCamera::GetHorizontalFOVWidth() -> int { return camera_->Width.GetValue(); };

auto BaslerCamera::GetMaxHorizontalWidth() -> int { return static_cast<int>(fov_.width); };

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
}  // namespace scanner::image_provider

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
  switch (static_cast<scanner::image_provider::BaslerCameraErrorCode>(error_code)) {
    case scanner::image_provider::BaslerCameraErrorCode::NO_ERROR:
      return "No error";
    case scanner::image_provider::BaslerCameraErrorCode::NO_CAMERA_FOUND:
      return "No camera found";
    case scanner::image_provider::BaslerCameraErrorCode::FAILED_TO_ACQUIRE_CAMERA:
      return "Failed to aquire camera";
    case scanner::image_provider::BaslerCameraErrorCode::FAILED_TO_SET_CONFIG:
      return "Failed to set configuration";
    case scanner::image_provider::BaslerCameraErrorCode::FAILED_TO_OPEN_CAMERA:
      return "Failed to open camera";
    case scanner::image_provider::BaslerCameraErrorCode::IMAGE_EVENT_HANDLER_NOT_SET:
      return "Image event handler was not set";
  }
}

auto ErrorCategory::default_error_condition(int other) const noexcept -> std::error_condition {
  switch (static_cast<scanner::image_provider::BaslerCameraErrorCode>(other)) {
    default:
      return {other, *this};
  }
}

const ErrorCategory ERROR_CATEGORY{};

}  // namespace

[[maybe_unused]] auto scanner::image_provider::make_error_code(BaslerCameraErrorCode error_code)
    -> std::error_code {  // NOLINT(*-identifier-naming)
  return {static_cast<int>(error_code), ERROR_CATEGORY};
}
