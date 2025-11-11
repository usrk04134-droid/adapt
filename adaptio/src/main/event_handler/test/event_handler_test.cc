
#include <doctest/doctest.h>
#include <prometheus/registry.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <nlohmann/json_fwd.hpp>
#include <optional>
#include <regex>
#include <string>
#include <vector>

#include "../src/event_handler_impl.h"
#include "../src/event_types.h"
#include "event_handler/event_codes.h"
#include "web_hmi/web_hmi.h"

using event::EventHandlerImpl;

// NOLINTBEGIN(readability-function-cognitive-complexity)

class WebHmiMock : public web_hmi::WebHmi {
 public:
  struct EventInfo {
    std::string code;
    std::string title;
    std::string ack_path;
    std::string status;
  };

  void Subscribe(std::string const& topic, web_hmi::OnRequest on_request) override {
    if (topic == "GetEvents") {
      get_events_handler = on_request;
    } else {
      FAIL("not implemented");
    }
  }
  void SubscribePattern(std::regex const& /*pattern*/, web_hmi::OnRequest on_request) override {
    web_hmi_on_requests = on_request;
  };
  void Send(nlohmann::json const& /*data*/) override { FAIL("not implemented"); }
  void Send(std::string const& topic, const std::optional<nlohmann::json>& /*result*/,
            const std::optional<nlohmann::json>& payload) override {
    if (topic == "events") {
      active_events.clear();
      for (const auto& event : *payload) {
        std::string code;
        std::string time;
        std::string title;
        std::string ack_path;
        std::string status;
        event.at("code").get_to(code);
        event.at("time").get_to(time);
        event.at("title").get_to(title);
        event.at("status").get_to(status);

        if (event.contains("ackPath")) {
          event.at("ackPath").get_to(ack_path);
        }

        CHECK_EQ(status, "active");
        CHECK_NE(code, "");
        CHECK_NE(time, "");
        CHECK_NE(title, "");

        active_events.push_back(EventInfo{
            .code     = code,
            .title    = title,
            .ack_path = ack_path,
            .status   = status,
        });
      }
    } else if (topic == "GetEventsRsp") {
      get_events_response.clear();
      for (const auto& event : *payload) {
        std::string code;
        std::string time;
        std::string title;
        std::string ack_path;
        std::string status;
        event.at("code").get_to(code);
        event.at("time").get_to(time);
        event.at("title").get_to(title);
        event.at("status").get_to(status);

        if (event.contains("ackPath")) {
          event.at("ackPath").get_to(ack_path);
        }

        CHECK_NE(code, "");
        CHECK_NE(time, "");
        CHECK_NE(title, "");

        get_events_response.push_back(EventInfo{
            .code     = code,
            .title    = title,
            .ack_path = ack_path,
            .status   = status,
        });
      }
    } else {
      FAIL("Unknown topic: {}", topic);
    }
  };
  void Send(std::string const& /*topic*/, nlohmann::json const& /*result*/,
            const std::optional<std::string>& /*message_status*/,
            const std::optional<nlohmann::json>& /*payload*/) override {
    FAIL("not implemented");
  };

  std::vector<EventInfo> active_events;
  std::vector<EventInfo> get_events_response;
  web_hmi::OnRequest web_hmi_on_requests;
  web_hmi::OnRequest get_events_handler;
};

TEST_SUITE("EventHandler") {
  TEST_CASE("EventAckAndWait") {
    WebHmiMock web_hmi{};
    auto registry = std::make_shared<prometheus::Registry>();
    EventHandlerImpl event_handler([]() { return std::chrono::system_clock::now(); }, {}, registry.get());

    event::Event const event{
        .version         = "1.0",
        .code            = "TestEvent",
        .title           = "Title",
        .description     = "Description",
        .action          = event::Action::QUICKSTOP,
        .resolution_type = event::ResolutionType::WAIT_AND_ACK,
    };

    event_handler.AddEvent(event);
    event_handler.SetWebHmi(&web_hmi);

    /* Raise the event */
    event_handler.RaiseEvent("TestEvent", std::nullopt);
    CHECK(web_hmi.active_events.size() == 1);
    CHECK(event_handler.ActiveBlockingEvents());

    /* Clear the event */
    event_handler.ClearEvent("TestEvent");
    CHECK(web_hmi.active_events.size() == 1);
    CHECK(event_handler.ActiveBlockingEvents());

    /* ACK the event */
    web_hmi.web_hmi_on_requests(web_hmi.active_events[0].ack_path, {});
    CHECK(web_hmi.active_events.empty());
    CHECK(!event_handler.ActiveBlockingEvents());

    /* Swap order of Clear and ACk */
    event_handler.RaiseEvent("TestEvent", std::nullopt);
    CHECK(web_hmi.active_events.size() == 1);
    CHECK(event_handler.ActiveBlockingEvents());
    web_hmi.web_hmi_on_requests(web_hmi.active_events[0].ack_path, {});
    CHECK(web_hmi.active_events.size() == 1);
    CHECK(event_handler.ActiveBlockingEvents());
    event_handler.ClearEvent("TestEvent");
    CHECK(web_hmi.active_events.empty());
    CHECK(!event_handler.ActiveBlockingEvents());
  }

  TEST_CASE("EventAckOrWait") {
    WebHmiMock web_hmi{};
    auto registry = std::make_shared<prometheus::Registry>();
    EventHandlerImpl event_handler([]() { return std::chrono::system_clock::now(); }, {}, registry.get());

    event::Event const event{
        .version         = "1.0",
        .code            = "TestEvent",
        .title           = "Title",
        .description     = "Description",
        .action          = event::Action::QUICKSTOP,
        .resolution_type = event::ResolutionType::WAIT_OR_ACK,
    };

    event_handler.AddEvent(event);
    event_handler.SetWebHmi(&web_hmi);

    /* Raise the event */
    event_handler.RaiseEvent("TestEvent", std::nullopt);
    CHECK(web_hmi.active_events.size() == 1);
    CHECK(event_handler.ActiveBlockingEvents());

    /* Clear the event */
    event_handler.ClearEvent("TestEvent");
    CHECK(web_hmi.active_events.size() == 1);
    CHECK(!event_handler.ActiveBlockingEvents());

    /* ACK the event */
    web_hmi.web_hmi_on_requests(web_hmi.active_events[0].ack_path, {});
    CHECK(web_hmi.active_events.empty());
    CHECK(!event_handler.ActiveBlockingEvents());

    /* Swap order of Clear and ACk */
    event_handler.RaiseEvent("TestEvent", std::nullopt);
    CHECK(web_hmi.active_events.size() == 1);
    CHECK(event_handler.ActiveBlockingEvents());
    web_hmi.web_hmi_on_requests(web_hmi.active_events[0].ack_path, {});
    CHECK(web_hmi.active_events.size() == 1);
    CHECK(!event_handler.ActiveBlockingEvents());
    event_handler.ClearEvent("TestEvent");
    CHECK(web_hmi.active_events.empty());
    CHECK(!event_handler.ActiveBlockingEvents());
  }

  TEST_CASE("Ackable") {
    WebHmiMock web_hmi{};
    auto registry = std::make_shared<prometheus::Registry>();
    EventHandlerImpl event_handler([]() { return std::chrono::system_clock::now(); }, {}, registry.get());

    event::Event const event{
        .version         = "1.0",
        .code            = "TestEvent",
        .title           = "Title",
        .description     = "Description",
        .action          = event::Action::QUICKSTOP,
        .resolution_type = event::ResolutionType::ACKABLE,
    };

    event_handler.AddEvent(event);
    event_handler.SetWebHmi(&web_hmi);

    /* Send event */
    event_handler.SendEvent("TestEvent", std::nullopt);
    CHECK(web_hmi.active_events.size() == 1);
    CHECK(!event_handler.ActiveBlockingEvents());

    web_hmi.web_hmi_on_requests(web_hmi.active_events[0].ack_path, {});
    CHECK(web_hmi.active_events.empty());
    CHECK(!event_handler.ActiveBlockingEvents());
  }

  TEST_CASE("RestartRequired") {
    WebHmiMock web_hmi{};
    auto registry = std::make_shared<prometheus::Registry>();
    EventHandlerImpl event_handler([]() { return std::chrono::system_clock::now(); }, {}, registry.get());

    event::Event const event{
        .version         = "1.0",
        .code            = "TestEvent",
        .title           = "Title",
        .description     = "Description",
        .action          = event::Action::QUICKSTOP,
        .resolution_type = event::ResolutionType::RESTART_REQUIRED,
    };

    event_handler.AddEvent(event);
    event_handler.SetWebHmi(&web_hmi);

    /* Send event */
    event_handler.SendEvent("TestEvent", std::nullopt);
    CHECK(web_hmi.active_events.size() == 1);
    CHECK(web_hmi.active_events[0].ack_path.empty());
    CHECK(event_handler.ActiveBlockingEvents());
  }

  TEST_CASE("Wait") {
    WebHmiMock web_hmi{};
    auto registry = std::make_shared<prometheus::Registry>();
    EventHandlerImpl event_handler([]() { return std::chrono::system_clock::now(); }, {}, registry.get());

    event::Event const event{
        .version         = "1.0",
        .code            = "TestEvent",
        .title           = "Title",
        .description     = "Description",
        .action          = event::Action::QUICKSTOP,
        .resolution_type = event::ResolutionType::WAIT,
    };

    event_handler.AddEvent(event);
    event_handler.SetWebHmi(&web_hmi);

    /* Raise the event */
    event_handler.RaiseEvent("TestEvent", std::nullopt);
    CHECK(web_hmi.active_events.size() == 1);
    CHECK(event_handler.ActiveBlockingEvents());
    CHECK(web_hmi.active_events[0].ack_path.empty());

    /* Clear the event */
    event_handler.ClearEvent("TestEvent");
    CHECK(web_hmi.active_events.empty());
    CHECK(!event_handler.ActiveBlockingEvents());
  }

  TEST_CASE("GetEvents") {
    WebHmiMock web_hmi{};
    auto registry = std::make_shared<prometheus::Registry>();
    EventHandlerImpl event_handler([]() { return std::chrono::system_clock::now(); }, {}, registry.get());

    auto const event1 = event::Event{
        .version         = "1.0",
        .code            = "TestEvent1",
        .title           = "Title1",
        .description     = "Description1",
        .action          = event::Action::QUICKSTOP,
        .resolution_type = event::ResolutionType::ACKABLE,
    };

    auto const event2 = event::Event{
        .version         = "1.0",
        .code            = "TestEvent2",
        .title           = "Title2",
        .description     = "Description2",
        .action          = event::Action::QUICKSTOP,
        .resolution_type = event::ResolutionType::ACKABLE,
    };

    event_handler.AddEvent(event1);
    event_handler.AddEvent(event2);
    event_handler.SetWebHmi(&web_hmi);

    /* Send first event */
    event_handler.SendEvent("TestEvent1", std::nullopt);
    CHECK(web_hmi.active_events.size() == 1);
    CHECK(web_hmi.active_events[0].code == "TestEvent1");
    CHECK(web_hmi.active_events[0].status == "active");

    /* Send second event */
    event_handler.SendEvent("TestEvent2", std::nullopt);
    CHECK(web_hmi.active_events.size() == 2);

    /* GetEvents should return both active events */
    web_hmi.get_events_handler("GetEvents", {});
    CHECK(web_hmi.get_events_response.size() == 2);
    CHECK(web_hmi.get_events_response[0].code == "TestEvent1");
    CHECK(web_hmi.get_events_response[0].status == "active");
    CHECK(web_hmi.get_events_response[1].code == "TestEvent2");
    CHECK(web_hmi.get_events_response[1].status == "active");

    /* Acknowledge first event - should move to resolved */
    web_hmi.web_hmi_on_requests(web_hmi.active_events[0].ack_path, {});
    CHECK(web_hmi.active_events.size() == 1);
    CHECK(web_hmi.active_events[0].code == "TestEvent2");

    /* GetEvents should return 1 active and 1 resolved event */
    web_hmi.get_events_handler("GetEvents", {});
    CHECK(web_hmi.get_events_response.size() == 2);

    /* Find active and resolved events in response */
    auto active_count   = 0;
    auto resolved_count = 0;
    for (const auto& event : web_hmi.get_events_response) {
      if (event.status == "active") {
        active_count++;
        CHECK(event.code == "TestEvent2");
      } else if (event.status == "resolved") {
        resolved_count++;
        CHECK(event.code == "TestEvent1");
      }
    }
    CHECK(active_count == 1);
    CHECK(resolved_count == 1);

    /* Acknowledge second event */
    web_hmi.web_hmi_on_requests(web_hmi.active_events[0].ack_path, {});
    CHECK(web_hmi.active_events.empty());

    /* GetEvents should return 0 active and 2 resolved events */
    web_hmi.get_events_handler("GetEvents", {});
    CHECK(web_hmi.get_events_response.size() == 2);

    for (const auto& event : web_hmi.get_events_response) {
      CHECK(event.status == "resolved");
    }
  }
}
// NOLINTEND(readability-function-cognitive-complexity)
