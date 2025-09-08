#pragma once

#include <prometheus/counter.h>
#include <prometheus/registry.h>

#include <chrono>
#include <filesystem>
#include <nlohmann/json_fwd.hpp>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "../event_handler.h"
#include "common/clock_functions.h"
#include "common/logging/component_logger.h"
#include "configuration/conf_file_handler.h"
#include "event_handler/event_codes.h"
#include "event_types.h"
#include "web_hmi/web_hmi.h"

namespace event {

class EventHandlerImpl : public EventHandler {
 public:
  explicit EventHandlerImpl(clock_functions::SystemClockNowFunc system_clock_now_func,
                            std::optional<std::filesystem::path> path_logs, prometheus::Registry* registry);
  auto LoadEventsFromFile(std::filesystem::path const& path) -> bool;
  void AddEvent(Event const& event); /* for test */
  void SetWebHmi(web_hmi::WebHmi* web_hmi);
  void RaiseEvent(const Code& code, std::optional<std::string> detail) override;
  void ClearEvent(const Code& code) override;
  void SendEvent(const Code& code, std::optional<std::string> detail) override;
  auto ActiveBlockingEvents() const -> bool;

 private:
  configuration::FileHandlerPtr fh_;
  std::unordered_map<std::string, Event> events_;
  web_hmi::WebHmi* web_hmi_{nullptr};
  clock_functions::SystemClockNowFunc system_clock_now_func_;
  std::optional<std::filesystem::path> path_logs_;
  prometheus::Registry* registry_;
  int64_t event_id_counter_{};
  common::logging::ComponentLogger event_logger_;

  struct {
    std::unordered_map<Code, prometheus::Counter*> raised_events;
  } metrics_;

  struct EventInternal {
    int64_t id;
    Code code;
    std::optional<std::string> detail{std::nullopt};
    std::chrono::system_clock::time_point timestamp;
    bool pending_clear;
    bool pending_ack;
    EventStatus status{EventStatus::ACTIVE};
  };
  std::vector<EventInternal> active_events_;
  std::vector<EventInternal> resolved_events_;

  auto EventInternalToJson(EventInternal const& ei) -> nlohmann::json;
  void SendEvents();
  void SendActiveAlarmsOnTimeout();
  void AckEvent(int64_t id);
  void SendEventInternal(const Event& event, const Code& code, std::optional<std::string> detail);
  void SetupMetrics(prometheus::Registry* registry);
  void LogEvent(const nlohmann::json& json);
  void LogCreatedEvent(const EventInternal& ei);
  void LogUpdatedEvent(const EventInternal& ei);
  void CleanupOldResolvedEvents();
};

}  // namespace event
