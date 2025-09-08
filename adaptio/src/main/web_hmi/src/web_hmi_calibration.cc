#include "web_hmi_calibration.h"

#include <nlohmann/json.hpp>
#include <nlohmann/json_fwd.hpp>
#include <string>
#include <utility>

#include "calibration/calibration_manager.h"
#include "calibration/calibration_types.h"
#include "common/logging/application_log.h"
#include "common/zevs/zevs_core.h"
#include "coordination/activity_status.h"
#include "joint_geometry/joint_geometry_provider.h"
#include "web_hmi/src/json_payload.h"
#include "web_hmi/web_hmi_json_helpers.h"

using web_hmi::WebHmiCalibration;
const std::string ADAPTIO_IO = "adaptio_io";

WebHmiCalibration::WebHmiCalibration(zevs::CoreSocket* socket, calibration::CalibrationManager* calibration_manager,
                                     joint_geometry::JointGeometryProvider* joint_geometry_provider,
                                     coordination::ActivityStatus* activity_status)
    : socket_(socket),
      calibration_manager_(calibration_manager),
      joint_geometry_provider_(joint_geometry_provider),
      activity_status_(activity_status) {
  calibration_manager->SetObserver(this);
}

void WebHmiCalibration::OnMessage(const std::string& message_name, const nlohmann::json& payload) {
  if (message_name == "LaserToTorchCalibration") {
    if (!activity_status_->IsIdle()) {
      // In this version do nothing except log
      LOG_ERROR("Cannot start LaserToTorchCalibration - busy with status: {}", activity_status_->ToString());
      return;
    }

    double offset                   = payload.at("offset").get<double>();
    double angle                    = payload.at("angle").get<double>();
    double stickout                 = payload.at("stickout").get<double>();
    auto calibration_joint_geometry = joint_geometry_provider_->GetFixtureJointGeometry();
    calibration_manager_->StartCalibrateLaserToTorch(calibration_joint_geometry, offset, angle, stickout);

    activity_status_->Set(coordination::ActivityStatusE::LASER_TORCH_CALIBRATION);

  } else if (message_name == "GetLaserToTorchCalibration") {
    auto calibration = calibration_manager_->GetLaserToTorchCalibration();
    bool valid       = false;
    if (calibration) {
      valid = true;
    } else {
      calibration = calibration::LaserTorchCalibration{};
    }
    auto payload = LaserTorchCalibrationToPayload(valid, calibration.value());
    auto message = CreateMessage("GetLaserToTorchCalibrationRsp", payload);
    socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));
  } else if (message_name == "SetLaserToTorchCalibration") {
    if (!activity_status_->IsIdle()) {
      // In this version do nothing except log
      LOG_ERROR("Cannot set LaserToTorchCalibration - busy with status: {}", activity_status_->ToString());
      return;
    }
    auto calibration = LaserTorchCalibrationFromPayload(payload);
    auto result      = calibration_manager_->SetLaserToTorchCalibration(calibration);
    auto payload     = ResultPayload(result);
    auto message     = CreateMessage("SetLaserToTorchCalibrationRsp", payload);
    socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));
  } else if (message_name == "WeldObjectCalibration") {
    if (!activity_status_->IsIdle()) {
      LOG_ERROR("Cannot start WeldObjectCalibration - busy with status: {}", activity_status_->ToString());
      auto payload = WeldObjectCalibrationToPayload(false, {});
      auto message = CreateMessage("WeldObjectCalibrationRsp", payload);
      socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));
      return;
    }

    double radius       = payload.at("radius").get<double>();
    double stickout     = payload.at("stickout").get<double>();
    auto joint_geometry = joint_geometry_provider_->GetJointGeometry();
    if (!joint_geometry.has_value()) {
      LOG_ERROR("Joint Geometry unavailable");
      return;
    }

    calibration_manager_->StartCalibrateWeldObject(joint_geometry.value(), radius, stickout);

    activity_status_->Set(coordination::ActivityStatusE::WELD_OBJECT_CALIBRATION);
  } else if (message_name == "GetWeldObjectCalibration") {
    auto calibration = calibration_manager_->GetWeldObjectCalibration();
    bool valid       = false;
    if (calibration) {
      valid = true;
    } else {
      calibration = calibration::WeldObjectCalibration{};
    }
    auto payload = WeldObjectCalibrationToPayload(valid, calibration.value());
    auto message = CreateMessage("GetWeldObjectCalibrationRsp", payload);
    socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));

  } else if (message_name == "SetWeldObjectCalibration") {
    auto calibration = WeldObjectCalibrationFromPayload(payload);
    auto result      = calibration_manager_->SetWeldObjectCalibration(calibration);
    auto payload     = ResultPayload(result);
    auto message     = CreateMessage("SetWeldObjectCalibrationRsp", payload);
    socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));

  } else {
    LOG_ERROR("Unknown Message: message_name={}", message_name);
  }
}

void WebHmiCalibration::LaserTorchCalibrationCompleted(const calibration::LaserTorchCalibration& data) {
  LOG_DEBUG("LaserToTorchCalibration completed");

  auto payload = LaserTorchCalibrationToPayload(true, data);
  auto message = CreateMessage("LaserToTorchCalibrationRsp", payload);
  socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));

  activity_status_->Set(coordination::ActivityStatusE::IDLE);
}

void WebHmiCalibration::LaserTorchCalibrationFailed() {
  LOG_DEBUG("LaserToTorchCalibration failed");

  auto payload = LaserTorchCalibrationToPayload(false, {});
  auto message = CreateMessage("LaserToTorchCalibrationRsp", payload);
  socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));

  activity_status_->Set(coordination::ActivityStatusE::IDLE);
}

void WebHmiCalibration::WeldObjectCalibrationCompleted(const calibration::WeldObjectCalibration& data) {
  LOG_DEBUG("WeldObjectCalibration completed");

  auto payload = WeldObjectCalibrationToPayload(true, data);
  auto message = CreateMessage("WeldObjectCalibrationRsp", payload);
  socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));

  activity_status_->Set(coordination::ActivityStatusE::IDLE);
}

void WebHmiCalibration::WeldObjectCalibrationFailed() {
  LOG_DEBUG("WeldObjectCalibration failed");

  auto payload = WeldObjectCalibrationToPayload(false, {});
  auto message = CreateMessage("WeldObjectCalibrationRsp", payload);
  socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));

  activity_status_->Set(coordination::ActivityStatusE::IDLE);
}
