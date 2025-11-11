#pragma once

#include <doctest/doctest.h>

#include "helpers.h"
#include "helpers_web_hmi.h"
#include "web_hmi/web_hmi_json_helpers.h"

// NOLINTBEGIN(*-magic-numbers, readability-function-cognitive-complexity)

inline auto StoreABPParams(TestFixture& fixture, nlohmann::json const& payload, bool expect_ok) {
  auto msg = web_hmi::CreateMessage("StoreABPParameters", std::nullopt, payload);
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));

  auto const response_payload = ReceiveJsonByName(fixture, "StoreABPParametersRsp");
  CHECK(response_payload != nullptr);

  auto const expected = expect_ok
    ? nlohmann::json { { "result", "ok"} }
    : nlohmann::json { { "result", "fail"} };

  CHECK_EQ(response_payload.at("result"), expected.at("result"));
}

inline auto StoreDefaultABPParams(TestFixture& fixture) {
  auto const payload = nlohmann::json({
      {"wallOffset",         3.0 },
      {"beadOverlap",        10.0},
      {"stepUpValue",        0.5 },
      {"kGain",              2.  },
      {"heatInput",
       {
           {"min", 2.1},
           {"max", 2.3},
       }                         },
      {"weldSystem2Current",
       {
           {"min", 675.0},
           {"max", 725.0},
       }                         },
      {"capCornerOffset",    1.5 },
      {"capBeads",           3   },
      {"capInitDepth",       7.0 }
  });
  StoreABPParams(fixture, payload, true);
}

inline auto CheckABPParamsEqual(TestFixture& fixture, nlohmann::json const& expected) {
  auto msg = web_hmi::CreateMessage("GetABPParameters", std::nullopt, {});
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));

  auto const response_payload = ReceiveJsonByName(fixture, "GetABPParametersRsp");

  CHECK(response_payload != nullptr);
  auto payload = response_payload.at("payload");
  REQUIRE(payload.at("wallOffset") == doctest::Approx(expected["wallOffset"]).epsilon(0.001));
  REQUIRE(payload.at("beadOverlap") == doctest::Approx(expected["beadOverlap"]).epsilon(0.001));
  REQUIRE(payload.at("stepUpValue") == doctest::Approx(expected["stepUpValue"]).epsilon(0.001));
  REQUIRE(payload.at("kGain") == doctest::Approx(expected["kGain"]).epsilon(0.001));
  REQUIRE(payload.at("heatInput").at("min") == doctest::Approx(expected["heatInput"]["min"]).epsilon(0.001));
  REQUIRE(payload.at("heatInput").at("max") == doctest::Approx(expected["heatInput"]["max"]).epsilon(0.001));
  REQUIRE(payload.at("weldSystem2Current").at("min") ==
          doctest::Approx(expected["weldSystem2Current"]["min"]).epsilon(0.001));
  REQUIRE(payload.at("weldSystem2Current").at("max") ==
          doctest::Approx(expected["weldSystem2Current"]["max"]).epsilon(0.001));
  REQUIRE(payload.at("beadSwitchAngle") == doctest::Approx(expected["beadSwitchAngle"]).epsilon(0.001));

  if (expected.contains("capCornerOffset")) {
    REQUIRE(payload.at("capCornerOffset") == doctest::Approx(expected["capCornerOffset"]).epsilon(0.001));
  }
  if (expected.contains("capBeads")) {
    REQUIRE(payload.at("capBeads") == expected["capBeads"]);
  }
  if (expected.contains("capInitDepth")) {
    REQUIRE(payload.at("capInitDepth") == doctest::Approx(expected["capInitDepth"]).epsilon(0.001));
  }
}

// NOLINTEND(*-magic-numbers, readability-function-cognitive-complexity)
