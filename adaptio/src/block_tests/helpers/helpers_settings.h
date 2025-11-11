#pragma once

#include <doctest/doctest.h>

#include <optional>
#include <string>

#include "helpers.h"
#include "helpers_web_hmi.h"
#include "web_hmi/web_hmi_json_helpers.h"

auto const USE_EDGE_SENSOR             = true;
auto const EDGE_SENSOR_PLACEMENT_VALUE = "right";
struct TestSettings {
  bool use_edge_sensor{USE_EDGE_SENSOR};
  std::string edge_sensor_placement{EDGE_SENSOR_PLACEMENT_VALUE};
};

inline auto TestSettingsToJson(TestSettings const& settings) -> nlohmann::json {
  return nlohmann::json({
      {"useEdgeSensor",       settings.use_edge_sensor      },
      {"edgeSensorPlacement", settings.edge_sensor_placement}
  });
}

inline auto StoreSettings(TestFixture& fixture, nlohmann::json const& payload, bool expect_ok) {
  auto msg = web_hmi::CreateMessage("SetSettings", std::nullopt, payload);
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));

  auto const response_payload = ReceiveJsonByName(fixture, "SetSettingsRsp");
  CHECK(response_payload != nullptr);

  auto const expected = expect_ok
    ? nlohmann::json { { "result", "ok"} }
    : nlohmann::json { { "result", "fail"} };

  CHECK_EQ(response_payload.at("result"), expected.at("result"));
}

inline auto StoreSettings(TestFixture& fixture, TestSettings const& settings, bool expect_ok) {
  StoreSettings(fixture, TestSettingsToJson(settings), expect_ok);
}

inline auto CheckSettingsEqual(TestFixture& fixture, nlohmann::json const& expected) {
  auto msg = web_hmi::CreateMessage("GetSettings", std::nullopt, {});
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));

  auto const response_payload = ReceiveJsonByName(fixture, "GetSettingsRsp");

  CHECK(response_payload != nullptr);
  auto payload = response_payload.at("payload");
  REQUIRE(payload.at("useEdgeSensor") == expected["useEdgeSensor"]);
  REQUIRE(payload.at("edgeSensorPlacement") == expected["edgeSensorPlacement"]);
}

inline auto CheckSettingsEqual(TestFixture& fixture, TestSettings const& expected) {
  CheckSettingsEqual(fixture, TestSettingsToJson(expected));
}
