#pragma once

#include "image_logging/image_logging_configuration.h"
#include "image_logging/image_logging_manager.h"
#include "scanner_client/scanner_client.h"
#include "web_hmi/web_hmi.h"
#include "weld_control/weld_state_observer.h"

namespace image_logging {

class ImageLoggingManagerImpl : public ImageLoggingManager, public weld_control::WeldStateObserver {
 public:
  explicit ImageLoggingManagerImpl(const Configuration& config, web_hmi::WebHmi* web_hmi,
                                   scanner_client::ScannerClient* scanner_client);

  ImageLoggingManagerImpl(ImageLoggingManagerImpl&)                     = delete;
  auto operator=(ImageLoggingManagerImpl&) -> ImageLoggingManagerImpl&  = delete;
  ImageLoggingManagerImpl(ImageLoggingManagerImpl&&)                    = delete;
  auto operator=(ImageLoggingManagerImpl&&) -> ImageLoggingManagerImpl& = delete;

  ~ImageLoggingManagerImpl() override = default;

  /* ImageLoggingManager interface */
  void Flush() override;

  /* WeldStateObserver interface */
  void OnWeldStateChanged(State state) override;

 private:
  void SetImageLoggingOnFromPayload(const nlohmann::json& payload);
  void UpdateImageLogger();

  Configuration default_config_;
  Configuration config_;
  web_hmi::WebHmi* web_hmi_;
  scanner_client::ScannerClient* scanner_client_;
  weld_control::WeldStateObserver::State weld_state_{weld_control::WeldStateObserver::State::IDLE};
};

}  // namespace image_logging
