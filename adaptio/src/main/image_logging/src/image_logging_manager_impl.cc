#include "image_logging_manager_impl.h"

#include <nlohmann/json.hpp>
#include <nlohmann/json_fwd.hpp>
#include <string>

#include "common/logging/application_log.h"
#include "image_logging/image_logging_configuration.h"
#include "scanner_client/scanner_client.h"
#include "web_hmi/web_hmi.h"
#include "weld_control/weld_state_observer.h"

namespace image_logging {

ImageLoggingManagerImpl::ImageLoggingManagerImpl(const Configuration& config, web_hmi::WebHmi* web_hmi,
                                                 scanner_client::ScannerClient* scanner_client)
    : default_config_(config), config_(config), web_hmi_(web_hmi), scanner_client_(scanner_client) {
  auto image_logging_on = [this](const std::string& /*name*/, const nlohmann::json& payload) {
    SetImageLoggingOnFromPayload(payload);
    this->UpdateImageLogger();
    web_hmi_->Send("ImageLoggingOnRsp", nlohmann::json{
                                            {"result", "ok"}
    });
  };

  auto image_logging_off = [this](const std::string& /*name*/, const nlohmann::json& /*payload*/) {
    config_.mode = Mode::OFF;
    this->UpdateImageLogger();
    web_hmi_->Send("ImageLoggingOffRsp", nlohmann::json{
                                             {"result", "ok"}
    });
  };

  auto image_logging_status = [this](const std::string& /*name*/, const nlohmann::json& /*payload*/) {
    web_hmi_->Send("ImageLoggingStatusRsp", nlohmann::json{
                                                {"mode",              ModeToString(config_.mode)       },
                                                {"sampleRate",        config_.sample_rate              },
                                                {"bufferSize",        config_.buffer_size              },
                                                {"path",              config_.path                     },
                                                {"onErrorIntervalMs", config_.on_error_interval.count()},
    });
  };

  auto image_logging_restore = [this](const std::string& /*name*/, const nlohmann::json& /*payload*/) {
    config_ = default_config_;
    this->UpdateImageLogger();
    web_hmi_->Send("ImageLoggingRestoreRsp", nlohmann::json{
                                                 {"result", "ok"}
    });
  };

  web_hmi_->Subscribe("ImageLoggingOn", image_logging_on);
  web_hmi_->Subscribe("ImageLoggingOff", image_logging_off);
  web_hmi_->Subscribe("ImageLoggingStatus", image_logging_status);
  web_hmi_->Subscribe("ImageLoggingRestore", image_logging_restore);

  UpdateImageLogger();
}

void ImageLoggingManagerImpl::SetImageLoggingOnFromPayload(const nlohmann::json& payload) {
  if (payload.contains(std::string("mode"))) {
    config_.mode = ModeFromString(payload.at("mode")).value_or(image_logging::Mode::OFF);
  }

  if (payload.contains(std::string{"sampleRate"})) {
    payload.at("sampleRate").get_to(config_.sample_rate);
  }

  if (payload.contains(std::string{"bufferSize"})) {
    payload.at("bufferSize").get_to(config_.buffer_size);
  }

  if (payload.contains("path") && !std::string(payload["path"]).empty()) {
    payload["path"].get_to(config_.path);
  }

  if (payload.contains("onErrorIntervalMs")) {
    auto on_error_interval_ms = 0;
    payload["onErrorIntervalMs"].get_to(on_error_interval_ms);

    config_.on_error_interval = std::chrono::milliseconds(on_error_interval_ms);
  }
}

void ImageLoggingManagerImpl::UpdateImageLogger() {
  LOG_INFO("config: {}", config_.ToString());

  auto data = scanner_client::ScannerClient::ImageLoggingData{
      .sample_rate       = config_.sample_rate,
      .buffer_size       = config_.buffer_size,
      .on_error_interval = config_.on_error_interval,
      .path              = config_.path,
  };

  switch (config_.mode) {
    case Mode::OFF:
      data.mode = scanner_client::ScannerClient::ImageLoggingData::Mode::OFF;
      break;
    case Mode::DIRECT:
      data.mode = scanner_client::ScannerClient::ImageLoggingData::Mode::DIRECT;
      break;
    case Mode::BUFFERED:
      data.mode = scanner_client::ScannerClient::ImageLoggingData::Mode::BUFFERED;
      break;
    case Mode::ON_ERROR:
      data.mode = scanner_client::ScannerClient::ImageLoggingData::Mode::ON_ERROR;
      break;
    case Mode::ON_ERROR_WELDING:
      data.mode = weld_state_ == weld_control::WeldStateObserver::State::WELDING
                      ? scanner_client::ScannerClient::ImageLoggingData::Mode::ON_ERROR
                      : scanner_client::ScannerClient::ImageLoggingData::Mode::OFF;
      break;
  }

  scanner_client_->ImageLoggingUpdate(data);
}

void ImageLoggingManagerImpl::Flush() { scanner_client_->FlushImageBuffer(); }

void ImageLoggingManagerImpl::OnWeldStateChanged(weld_control::WeldStateObserver::State state) {
  weld_state_ = state;
  UpdateImageLogger();
}

}  // namespace image_logging
