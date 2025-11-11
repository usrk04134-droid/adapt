#pragma once

#include <functional>
#include <nlohmann/json.hpp>
#include <regex>

namespace web_hmi {

using OnRequest = std::function<void(std::string const& topic, nlohmann::json const& payload)>;

class WebHmi {
 public:
  virtual ~WebHmi() = default;

  virtual void Subscribe(std::string const& topic, OnRequest on_request)                                            = 0;
  virtual void SubscribePattern(std::regex const& pattern, OnRequest on_request)                                    = 0;
  virtual void Send(nlohmann::json const& data)                                                                     = 0;
  virtual void Send(std::string const& topic, const std::optional<nlohmann::json>& result,
                    const std::optional<nlohmann::json>& payload)                                                   = 0;
  virtual void Send(std::string const& topic, nlohmann::json const& result,
                    const std::optional<std::string>& message_status, const std::optional<nlohmann::json>& payload) = 0;
};

}  // namespace web_hmi
