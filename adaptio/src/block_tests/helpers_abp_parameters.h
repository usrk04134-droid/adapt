#pragma once

#include <doctest/doctest.h>

#include "block_tests/helpers_web_hmi.h"
#include "helpers.h"
#include "web_hmi/web_hmi_json_helpers.h"

// NOLINTBEGIN(*-magic-numbers, readability-function-cognitive-complexity)

inline auto StoreABPParams(TestFixture& fixture, nlohmann::json const& payload, bool expect_ok) {
  auto msg = web_hmi::CreateMessage("StoreABPParameters", payload);
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));

  auto const response_payload = ReceiveJsonByName(fixture, "StoreABPParametersRsp");
  CHECK(response_payload != nullptr);

  auto const expected = expect_ok
    ? nlohmann::json { { "result", "ok"} }
    : nlohmann::json { { "result", "fail"} };

  CHECK_EQ(response_payload, expected);
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
  auto msg = web_hmi::CreateMessage("GetABPParameters", {});
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));

  auto const response_payload = ReceiveJsonByName(fixture, "GetABPParametersRsp");

  CHECK(response_payload != nullptr);
  REQUIRE(response_payload["wallOffset"] == doctest::Approx(expected["wallOffset"]).epsilon(0.001));
  REQUIRE(response_payload["beadOverlap"] == doctest::Approx(expected["beadOverlap"]).epsilon(0.001));
  REQUIRE(response_payload["stepUpValue"] == doctest::Approx(expected["stepUpValue"]).epsilon(0.001));
  REQUIRE(response_payload["kGain"] == doctest::Approx(expected["kGain"]).epsilon(0.001));
  REQUIRE(response_payload["heatInput"]["min"] == doctest::Approx(expected["heatInput"]["min"]).epsilon(0.001));
  REQUIRE(response_payload["heatInput"]["max"] == doctest::Approx(expected["heatInput"]["max"]).epsilon(0.001));
  REQUIRE(response_payload["weldSystem2Current"]["min"] ==
          doctest::Approx(expected["weldSystem2Current"]["min"]).epsilon(0.001));
  REQUIRE(response_payload["weldSystem2Current"]["max"] ==
          doctest::Approx(expected["weldSystem2Current"]["max"]).epsilon(0.001));
  REQUIRE(response_payload["beadSwitchAngle"] == doctest::Approx(expected["beadSwitchAngle"]).epsilon(0.001));

  if (expected.contains("capCornerOffset")) {
    REQUIRE(response_payload["capCornerOffset"] == doctest::Approx(expected["capCornerOffset"]).epsilon(0.001));
  }
  if (expected.contains("capBeads")) {
    REQUIRE(response_payload["capBeads"] == expected["capBeads"]);
  }
  if (expected.contains("capInitDepth")) {
    REQUIRE(response_payload["capInitDepth"] == doctest::Approx(expected["capInitDepth"]).epsilon(0.001));
  }
}

// NOLINTEND(*-magic-numbers, readability-function-cognitive-complexity)
