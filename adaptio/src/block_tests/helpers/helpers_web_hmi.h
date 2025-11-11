#pragma once
#include <doctest/doctest.h>

#include <nlohmann/json_fwd.hpp>

#include "common/zevs/zevs_test_support.h"
#include "helpers.h"

inline const auto SUCCESS_PAYLOAD = nlohmann::json{
    {"result", "ok"}
};

inline const auto FAILURE_PAYLOAD = nlohmann::json{
    {"result", "fail"}
};

inline auto UnpackMessageName(const zevs::MessagePtr& message) -> std::string {
  auto jstr               = std::string{static_cast<char*>(message->Data()), message->Size()};
  nlohmann::json json_obj = nlohmann::json::parse(jstr);

  auto message_name = json_obj.at("name").get<std::string>();
  return message_name;
}

inline auto UnpackMessage(const zevs::MessagePtr& message) -> nlohmann::json {
  auto jstr               = std::string{static_cast<char*>(message->Data()), message->Size()};
  nlohmann::json json_obj = nlohmann::json::parse(jstr);

  nlohmann::json result;
  if (json_obj.contains("result")) {
    result = json_obj.at("result");
  }

  nlohmann::json payload;
  if (json_obj.contains("payload")) {
    payload = json_obj.at("payload");
  }

  nlohmann::json result_json = {
      {"result",  result },
      {"payload", payload}
  };

  if (json_obj.contains("message")) {
    result_json["message"] = json_obj.at("message");
  }
  return result_json;
}

inline auto ReceiveJsonByName(TestFixture& fixture, const std::string& name) -> nlohmann::json {
  const auto fn_match = [name](const zevs::MessagePtr& message) {
    auto const name0 = UnpackMessageName(message);

    return name0 == name;
  };

  auto rec_msg = fixture.WebHmiOut()->Receive(fn_match);
  DOCTEST_CHECK_MESSAGE(rec_msg, "Failed to receive message with name: " << name);
  return rec_msg ? UnpackMessage(rec_msg) : nlohmann::json{};
}
