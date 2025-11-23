#pragma once

#include <memory>
#include <regex>

#include "../web_hmi.h"
#include "calibration/src/calibration_metrics.h"
#include "common/zevs/zevs_core.h"
#include "coordination/activity_status.h"
#include "joint_geometry/joint_geometry_provider.h"
#include "kinematics/kinematics_client.h"

namespace web_hmi {

class WebHmiServer : public WebHmi {
 public:
  WebHmiServer(zevs::CoreSocket* in_socket, zevs::CoreSocket* out_socket,
               joint_geometry::JointGeometryProvider* joint_geometry_provider,
               kinematics::KinematicsClient* kinematics_client, coordination::ActivityStatus* activity_status,
               calibration::CalibrationMetrics* calibration_metrics);

  void OnMessage(zevs::MessagePtr message);

  // WebHmi interface
  void Subscribe(std::string const& topic, OnRequest on_request) override;
  void SubscribePattern(std::regex const& pattern, OnRequest on_request) override;
  void Send(nlohmann::json const& data) override;
  void Send(std::string const& topic, const std::optional<nlohmann::json>& result,
            const std::optional<nlohmann::json>& payload) override;
  void Send(std::string const& topic, nlohmann::json const& result, const std::optional<std::string>& message_status,
            const std::optional<nlohmann::json>& payload) override;

 private:
  void GetSlidesPositionRsp(double horizontal, double vertical);

  zevs::CoreSocket* in_socket_;
  zevs::CoreSocket* out_socket_;
  kinematics::KinematicsClient* kinematics_client_;
  coordination::ActivityStatus* activity_status_;
  calibration::CalibrationMetrics* calibration_metrics_;

  auto CheckSubscribers(std::string const& topic, nlohmann::json const& payload) -> bool;

  struct Subscriber {
    std::regex pattern;
    OnRequest on_request;
  };
  std::vector<Subscriber> subscribers_;
};

}  // namespace web_hmi
