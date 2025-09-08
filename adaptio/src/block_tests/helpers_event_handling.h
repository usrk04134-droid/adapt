#pragma once

#include <doctest/doctest.h>
#include <SQLiteCpp/Database.h>
#include <SQLiteCpp/Statement.h>
#include <SQLiteCpp/VariadicBind.h>

#include <string>
#include <vector>

#include "event_handler/event_codes.h"
#include "helpers.h"
#include "web_hmi/web_hmi_json_helpers.h"

// NOLINTBEGIN(readability-function-cognitive-complexity)

void inline CheckEvents(TestFixture& fixture, std::vector<event::Event> expected_events) {
  std::string name;
  nlohmann::json payload;
  web_hmi::UnpackMessage(fixture.WebHmiOut()->ReceiveMessage(), name, payload);

  CHECK_EQ(name, "events");

  bool acked = false;
  for (nlohmann::json const& event : payload) {
    auto it = std::find_if(expected_events.begin(), expected_events.end(),
                           [event, &fixture, &acked](const event::Event& expected_event) {
                             std::string code;
                             std::string time;
                             event.at("code").get_to(code);
                             event.at("time").get_to(time);

                             if (code == expected_event.code) {
                               CHECK_NE(time, "");

                               if (event.contains("ackPath")) {
                                 std::string ack_path;
                                 event.at("ackPath").get_to(ack_path);
                                 LOG_INFO("Test: ack event: {} path: {}", code, ack_path);
                                 fixture.WebHmiIn()->DispatchMessage(web_hmi::CreateMessage(ack_path, {}));
                                 acked = true;
                               }

                               return true;
                             }

                             return false;
                           });

    if (it != expected_events.end()) {
      expected_events.erase(it);
    } else {
      LOG_ERROR("{}", payload.dump());
      FAIL("Event not expected: ", event["code"], " ", event["title"]);
    }
  }

  for (auto const& event : expected_events) {
    FAIL("Expected event not triggered: ", event.code, " ", event.title);
  }

  if (acked) {
    // ACKed events will send an updated event list
    CheckEvents(fixture, {});
  }
}
// NOLINTEND(readability-function-cognitive-complexity)
