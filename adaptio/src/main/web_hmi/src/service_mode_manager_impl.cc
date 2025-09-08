#include "service_mode_manager_impl.h"

#include <cstdint>
#include <memory>
#include <nlohmann/json_fwd.hpp>
#include <string>

#include "common/logging/application_log.h"
#include "common/zevs/zevs_core.h"
#include "coordination/activity_status.h"
#include "joint_geometry/joint_geometry_provider.h"
#include "kinematics/kinematics_client.h"
#include "service_mode_tracking.h"
#include "web_hmi/web_hmi.h"
#include "weld_control/weld_control.h"

using kinematics::KinematicsClient;

using web_hmi::ServiceModeManagerImpl;

ServiceModeManagerImpl::ServiceModeManagerImpl(zevs::CoreSocket* socket, KinematicsClient* kinematics_client,
                                               joint_geometry::JointGeometryProvider* join_geometry_provider,
                                               weld_control::WeldControl* weld_control,
                                               coordination::ActivityStatus* activity_status, WebHmi* web_hmi)
    : socket_(socket),
      kinematics_client_(kinematics_client),
      joint_geometry_provider_(join_geometry_provider),
      weld_control_(weld_control),
      activity_status_(activity_status),
      web_hmi_(web_hmi) {
  auto service_mode_stop = [this](std::string const& /*message_name*/, const nlohmann::json& /*payload*/) {
    this->Stop();
  };

  auto service_mode_kinematics_control = [this](std::string const& /*message_name*/,
                                                const nlohmann::json& /*payload*/) {
    if (!activity_status_->IsIdle()) {
      LOG_DEBUG("Cannot start ServiceModeKinematicsControl - busy with status:{}",
                static_cast<uint32_t>(activity_status_->Get()));
      return;
    }

    // Changed temporarily to simplify integration of calibration v2
    activity_status_->Set(coordination::ActivityStatusE::CALIBRATION_AUTO_MOVE);
  };

  auto set_slides_position = [this](std::string const& /*message_name*/, const nlohmann::json& payload) {
    auto const default_slide_horizontal_lin_velocity = 3.667;
    auto const default_vertical_velocity             = 3.667;
    double horizontal                                = payload.at("horizontal").get<double>();
    double vertical                                  = payload.at("vertical").get<double>();
    double horizontal_velocity = payload.contains("horizontalVelocity") ? payload.at("horizontalVelocity").get<double>()
                                                                        : default_slide_horizontal_lin_velocity;
    double vertical_velocity =
        payload.contains("verticalVelocity") ? payload.at("verticalVelocity").get<double>() : default_vertical_velocity;
    kinematics_client_->SetSlidesPosition(horizontal, vertical, horizontal_velocity, vertical_velocity);
  };

  auto set_weld_axis_data = [this](std::string const& /*message_name*/, const nlohmann::json& payload) {
    auto const default_velocity = 0.;
    double velocity = payload.contains("velocity") ? payload.at("velocity").get<double>() : default_velocity;
    kinematics_client_->SetWeldAxisData(velocity);
  };

  auto service_mode_tracking = [this](std::string const& /*message_name*/, const nlohmann::json& /*payload*/) {
    if (!activity_status_->IsIdle()) {
      LOG_DEBUG("Cannot start ServiceModeTracking - busy  with status:{}",
                static_cast<uint32_t>(activity_status_->Get()));
      return;
    }
    service_ = std::make_unique<ServiceModeTracking>(weld_control_, socket_, joint_geometry_provider_);

    activity_status_->Set(coordination::ActivityStatusE::SERVICE_MODE_TRACKING);
  };

  auto service_mode_forward = [this](std::string const& message_name, const nlohmann::json& payload) {
    if (service_) {
      service_->OnMessage(message_name, payload);
    } else {
      LOG_INFO("No service started. Unhandled message: {}", message_name);
    }
  };

  web_hmi_->Subscribe("ServiceModeStop", service_mode_stop);
  web_hmi_->Subscribe("ServiceModeKinematicsControl", service_mode_kinematics_control);
  web_hmi_->Subscribe("ServiceModeTracking", service_mode_tracking);
  web_hmi_->Subscribe("SetSlidesPosition", set_slides_position);
  web_hmi_->Subscribe("StartTracking", service_mode_forward);
  web_hmi_->Subscribe("SetWeldAxisData", set_weld_axis_data);
}

void ServiceModeManagerImpl::Stop() {
  service_.reset();
  if (activity_status_->Get() == coordination::ActivityStatusE::CALIBRATION_AUTO_MOVE ||
      activity_status_->Get() == coordination::ActivityStatusE::SERVICE_MODE_TRACKING) {
    activity_status_->Set(coordination::ActivityStatusE::IDLE);
  }
}
