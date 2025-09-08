#pragma once

#include <nlohmann/json.hpp>
#include <nlohmann/json_fwd.hpp>

namespace web_hmi {

class Service {
 public:
  virtual ~Service() = default;

  virtual void OnMessage(const std::string& message_name, const nlohmann::json& payload) = 0;
};

}  // namespace web_hmi
