#pragma once

#include <nlohmann/json.hpp>
#include <nlohmann/json_fwd.hpp>

#include "common/logging/application_log.h"
#include "common/zevs/zevs_core.h"

namespace web_hmi {

inline auto CreateMessage(const std::string& message_name, const std::optional<nlohmann::json>& result,
                          const std::optional<nlohmann::json>& payload) -> zevs::MessagePtr {
  nlohmann::json json_obj;
  json_obj = {
      {"name", message_name},
  };

  if (result) {
    json_obj["result"] = result->value("result", "");
  }

  if (!payload || payload->is_null()) {
    json_obj["payload"] = nlohmann::json::object();
  } else {
    json_obj["payload"] = *payload;
  }

  auto jstr = json_obj.dump();
  LOG_DEBUG("web_hmi::PackMessage: {}", jstr.c_str());

  auto message = zevs::GetCoreFactory()->CreateRawMessage(jstr.size());
  std::memcpy(message->Data(), jstr.c_str(), jstr.size());

  return message;
}

inline auto CreateMessage(const std::string& message_name, nlohmann::json const& result,
                          const std::optional<std::string>& message_status,
                          const std::optional<nlohmann::json>& payload) -> zevs::MessagePtr {
  nlohmann::json json_obj = {
      {"name", message_name},
      {"result", result.value("result", "")}
  };

  if (result.value("result", "") == "ok") {
    if (!payload || payload->is_null()) {
      json_obj["payload"] = nlohmann::json::object();
    } else {
      json_obj["payload"] = *payload;
    }
  } else {
    if (message_status) {
      json_obj["message"] = *message_status;
    }
  }
  auto jstr = json_obj.dump();
  LOG_DEBUG("web_hmi::PackMessage: {}", jstr.c_str());

  auto message = zevs::GetCoreFactory()->CreateRawMessage(jstr.size());
  std::memcpy(message->Data(), jstr.c_str(), jstr.size());

  return message;
}

inline void UnpackMessage(const zevs::MessagePtr& message, std::string& message_name, nlohmann::json& payload) {
  if (message) {
    auto jstr               = std::string{static_cast<char*>(message->Data()), message->Size()};
    nlohmann::json json_obj = nlohmann::json::parse(jstr);
    LOG_DEBUG("web_hmi::UnpackMessage: {}", jstr.c_str());

    message_name = json_obj.at("name").get<std::string>();
    payload      = json_obj.at("payload");
  } else {
    message_name = "";
    payload      = "";
  }
}

inline auto UnpackMessagePayload(const zevs::MessagePtr& message) -> nlohmann::json {
  auto jstr               = std::string{static_cast<char*>(message->Data()), message->Size()};
  nlohmann::json json_obj = nlohmann::json::parse(jstr);

  auto payload = json_obj.at("payload");
  return payload;
}

}  // namespace web_hmi
