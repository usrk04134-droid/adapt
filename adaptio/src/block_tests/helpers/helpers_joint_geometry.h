#pragma once

#include <doctest/doctest.h>

#include <optional>

#include "helpers.h"
#include "helpers_web_hmi.h"
#include "web_hmi/web_hmi_json_helpers.h"

// NOLINTBEGIN(*-magic-numbers, readability-function-cognitive-complexity)

inline auto StoreJointGeometryParams(TestFixture& fixture, nlohmann::json const& payload, bool expect_ok) {
  auto msg = web_hmi::CreateMessage("SetJointGeometry", std::nullopt, payload);
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));

  auto response_payload = ReceiveJsonByName(fixture, "SetJointGeometryRsp");
  CHECK(response_payload != nullptr);

  auto const expected = expect_ok
    ? nlohmann::json { { "result", "ok"} }
    : nlohmann::json { { "result", "fail"} };

  CHECK_EQ(response_payload.at("result"), expected.at("result"));
}

inline auto StoreDefaultJointGeometryParams(TestFixture& fixture) {
  auto const payload = nlohmann::json({
      {"upperJointWidthMm",       57.58 },
      {"grooveDepthMm",           19.6  },
      {"leftJointAngleRad",       0.5236},
      {"rightJointAngleRad",      0.5236},
      {"leftMaxSurfaceAngleRad",  0.3491},
      {"rightMaxSurfaceAngleRad", 0.3491}
  });
  StoreJointGeometryParams(fixture, payload, true);
}

inline auto CheckJointGeometryParamsEqual(TestFixture& fixture, nlohmann::json const& expected) {
  auto msg = web_hmi::CreateMessage("GetJointGeometry", std::nullopt, {});
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));

  auto const response_payload = ReceiveJsonByName(fixture, "GetJointGeometryRsp");

  CHECK(response_payload != nullptr);
  auto payload = response_payload.at("payload");

  REQUIRE(payload.at("upperJointWidthMm") == doctest::Approx(expected["upperJointWidthMm"]).epsilon(0.001));
  REQUIRE(payload.at("grooveDepthMm") == doctest::Approx(expected["grooveDepthMm"]).epsilon(0.001));
  REQUIRE(payload.at("leftJointAngleRad") == doctest::Approx(expected["leftJointAngleRad"]).epsilon(0.001));
  REQUIRE(payload.at("rightJointAngleRad") == doctest::Approx(expected["rightJointAngleRad"]).epsilon(0.001));
  REQUIRE(payload.at("leftMaxSurfaceAngleRad") == doctest::Approx(expected["leftMaxSurfaceAngleRad"]).epsilon(0.001));
  REQUIRE(payload.at("rightMaxSurfaceAngleRad") == doctest::Approx(expected["rightMaxSurfaceAngleRad"]).epsilon(0.001));
}

// NOLINTEND(*-magic-numbers, readability-function-cognitive-complexity)
