#include "service_mode_tracking.h"

#include <nlohmann/json.hpp>
#include <nlohmann/json_fwd.hpp>
#include <string>
#include <utility>

#include "../web_hmi_json_helpers.h"
#include "common/logging/application_log.h"
#include "common/zevs/zevs_core.h"
#include "joint_geometry/joint_geometry_provider.h"
#include "tracking/tracking_manager.h"
#include "weld_control/weld_control.h"

namespace {

const std::string ADAPTIO_IO = "adaptio_io";

}  // namespace

using web_hmi::ServiceModeTracking;

ServiceModeTracking::ServiceModeTracking(weld_control::WeldControl* weld_control, zevs::CoreSocket* socket,
                                         joint_geometry::JointGeometryProvider* joint_geometry_provider)
    : weld_control_(weld_control),
      original_observer_(weld_control_->GetObserver()),
      socket_(socket),
      joint_geometry_provider_(joint_geometry_provider) {
  weld_control_->SetObserver(this);
}

ServiceModeTracking::~ServiceModeTracking() {
  weld_control_->Stop();
  weld_control_->SetObserver(original_observer_);
}

void ServiceModeTracking::OnNotifyHandoverToManual() {
  LOG_INFO("Service Mode Tracking Stopped - Groove too shallow");
  weld_control_->Stop();
  auto message = CreateMessage("TrackingGrooveShallow", {});
  socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));
}

void ServiceModeTracking::OnGrooveDataTimeout() {
  LOG_INFO("Service Mode Tracking Stopped - Groove data timeout");
  weld_control_->Stop();
  auto message = CreateMessage("TrackingGrooveDataTimeout", {});
  socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));
}

void ServiceModeTracking::OnError() { LOG_DEBUG("OnError"); }
void ServiceModeTracking::OnGracefulStop() { LOG_DEBUG("OnGracefulStop"); }

void ServiceModeTracking::OnMessage(const std::string& message_name, const nlohmann::json& payload) {
  if (message_name == "StartTracking") {
    auto joint_geometry = joint_geometry_provider_->GetJointGeometry();
    if (!joint_geometry.has_value()) {
      LOG_ERROR("Service Mode TrackingJointGeometry not available");
      auto message = CreateMessage("TrackingOperationDataInvalid", {});
      socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));
      return;
    }

    double horizontal_offset = payload.at("horizontal_offset").get<double>();
    double vertical_offset   = payload.at("vertical_offset").get<double>();

    weld_control_->JointTrackingStart(joint_geometry.value(), tracking::TrackingMode::TRACKING_LEFT_HEIGHT,
                                      horizontal_offset, vertical_offset);
    LOG_DEBUG("Service Mode Tracking started");
  } else {
    LOG_ERROR("Unknown Message: message_name={}", message_name);
  }
}
