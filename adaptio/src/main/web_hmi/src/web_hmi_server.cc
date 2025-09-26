#include "web_hmi_server.h"

#include <cstdint>
#include <cstring>
#include <memory>
#include <nlohmann/json.hpp>
#include <nlohmann/json_fwd.hpp>
#include <regex>
#include <string>
#include <utility>

#include "../web_hmi_json_helpers.h"
#include "calibration/calibration_metrics.h"
#include "common/logging/application_log.h"
#include "common/zevs/zevs_core.h"
#include "coordination/activity_status.h"
#include "joint_geometry/joint_geometry_provider.h"
#include "cli_handler/json_payload.h"
#include "kinematics/kinematics_client.h"
#include "lpcs/lpcs_slice.h"
#include "macs/macs_point.h"
#include "macs/macs_slice.h"
#include "version.h"
#include "web_hmi/web_hmi.h"

namespace {

const std::string ADAPTIO_IO = "adaptio_io";

}  // namespace

using web_hmi::WebHmiServer;

WebHmiServer::WebHmiServer(zevs::CoreSocket* in_socket, zevs::CoreSocket* out_socket,
                           joint_geometry::JointGeometryProvider* joint_geometry_provider,
                           kinematics::KinematicsClient* kinematics_client,
                           coordination::ActivityStatus* activity_status,
                           calibration::CalibrationMetrics* calibration_metrics)
    : in_socket_(in_socket),
      out_socket_(out_socket),
      kinematics_client_(kinematics_client),
      activity_status_(activity_status),
      calibration_metrics_(calibration_metrics) {
  LOG_DEBUG("Starting WebHmiServer");
  auto handler = [this](zevs::MessagePtr msg) { this->OnMessage(std::move(msg)); };
  in_socket_->SetHandler(handler);
}

auto WebHmiServer::CheckSubscribers(std::string const& topic, nlohmann::json const& payload) -> bool {
  bool found = false;
  for (const Subscriber& sub : subscribers_) {
    if (std::regex_match(topic, sub.pattern)) {
      sub.on_request(topic, payload);
      found = true;
    }
  }
  return found;
}

void WebHmiServer::OnMessage(zevs::MessagePtr message) {
  std::string message_name;

  try {
    nlohmann::json payload;
    UnpackMessage(message, message_name, payload);

    if (CheckSubscribers(message_name, payload)) {
      return;
    }

    if (message_name == "GetAdaptioVersion") {
      auto response = CreateMessage("GetAdaptioVersionRsp", VersionToPayload(ADAPTIO_VERSION));
      out_socket_->SendWithEnvelope(ADAPTIO_IO, std::move(response));

    } else if (message_name == "GetSlidesPosition") {
      auto on_get_slides_position = [this](std::uint64_t /*time_stamp*/, double horizontal, double vertical) {
        auto payload = PositionToPayload(horizontal, vertical);
        auto message = CreateMessage("GetSlidesPositionRsp", payload);
        out_socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));
      };
      kinematics_client_->GetSlidesPosition(on_get_slides_position);
    } else if (message_name == "GetSlidesStatus") {
      auto on_get_slides_status = [this](bool horizontal_in_position, bool vertical_in_position) {
        auto payload = SlidesStatusToPayload(horizontal_in_position, vertical_in_position);
        auto message = CreateMessage("GetSlidesStatusRsp", payload);
        out_socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));
      };
      kinematics_client_->GetSlidesStatus(on_get_slides_status);
    } else if (message_name == "GetActivityStatus") {
      auto payload = ActivityStatusToPayload(activity_status_->Get());
      auto message = CreateMessage("GetActivityStatusRsp", payload);
      out_socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));

    } else if (message_name == "GetGroove") {
      nlohmann::json payload;
      if (groove_) {
        payload = GrooveToPayload(groove_.value());
      }
      auto message = CreateMessage("GetGrooveRsp", payload);
      out_socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));
    } else if (message_name == "GetEdgePosition") {
      auto on_get_edge_position = [this](double position) {
        nlohmann::json payload = {
            {"position", position}
        };
        auto message = CreateMessage("GetEdgePositionRsp", payload);
        out_socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));
      };
      kinematics_client_->GetEdgePosition(on_get_edge_position);
    } else {
      LOG_ERROR("Unknown Message: message_name={}", message_name);
    }
  } catch (nlohmann::json::exception& exception) {
    LOG_ERROR("nlohman JSON exception: {}, for message: {}", exception.what(), message_name);
  }
}

// SliceObserver
void WebHmiServer::Receive(const macs::Slice& data, const lpcs::Slice& /*scanner_data*/,
                           const macs::Point& /*axis_position*/, const double /*angle_from_torch_to_scanner*/) {
  // could be an empty optional
  groove_ = data.groove;
}

void WebHmiServer::SubscribePattern(std::regex const& pattern, OnRequest on_request) {
  /* allow for multiple subscribers for the same topic */
  const Subscriber sub = {
      .pattern    = pattern,
      .on_request = std::move(on_request),
  };

  subscribers_.push_back((sub));
}

void WebHmiServer::Subscribe(std::string const& topic, OnRequest on_request) {
  SubscribePattern(std::regex(topic), on_request);
}

void WebHmiServer::Send(nlohmann::json const& data) {
  auto jstr = data.dump();

  LOG_TRACE("web_hmi::Send data: {}", jstr.c_str());

  auto message = zevs::GetCoreFactory()->CreateRawMessage(jstr.size());
  std::memcpy(message->Data(), jstr.c_str(), jstr.size());
  out_socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));
}

void WebHmiServer::Send(std::string const& topic, nlohmann::json const& payload) {
  LOG_TRACE("web_hmi::Send topic: {} payload: {}", topic.c_str(), payload.dump());

  auto message = CreateMessage(topic, payload);
  out_socket_->SendWithEnvelope(ADAPTIO_IO, std::move(message));
}
