#pragma once

#include <doctest/doctest.h>

#include "block_tests/helpers_web_hmi.h"
#include "helpers.h"
#include "web_hmi/web_hmi_json_helpers.h"

// NOLINTBEGIN(*-magic-numbers, readability-function-cognitive-complexity)

inline auto StoreJointGeometryParams(TestFixture& fixture, nlohmann::json const& payload, bool expect_ok) {
  auto msg = web_hmi::CreateMessage("SetJointGeometry", payload);
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));

  auto response_payload = ReceiveJsonByName(fixture, "SetJointGeometryRsp");
  CHECK(response_payload != nullptr);

  auto const expected = expect_ok
    ? nlohmann::json { { "result", "ok"} }
    : nlohmann::json { { "result", "fail"} };

  CHECK_EQ(response_payload, expected);
}

inline auto StoreDefaultJointGeometryParams(TestFixture& fixture) {
  auto const payload = nlohmann::json({
      {"upper_joint_width_mm",        50.0    },
      {"groove_depth_mm",             28.0    },
      {"left_joint_angle_rad",        0.1396  },
      {"right_joint_angle_rad",       0.1396  },
      {"left_max_surface_angle_rad",  0.349066},
      {"right_max_surface_angle_rad", 0.349066}
  });
  StoreJointGeometryParams(fixture, payload, true);
}

inline auto CheckJointGeometryParamsEqual(TestFixture& fixture, nlohmann::json const& expected) {
  auto msg = web_hmi::CreateMessage("GetJointGeometry", {});
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));

  auto const response_payload = ReceiveJsonByName(fixture, "GetJointGeometryRsp");

  REQUIRE(response_payload.at("upper_joint_width_mm") ==
          doctest::Approx(expected["upper_joint_width_mm"]).epsilon(0.001));
  REQUIRE(response_payload.at("groove_depth_mm") == doctest::Approx(expected["groove_depth_mm"]).epsilon(0.001));
  REQUIRE(response_payload.at("left_joint_angle_rad") ==
          doctest::Approx(expected["left_joint_angle_rad"]).epsilon(0.001));
  REQUIRE(response_payload.at("right_joint_angle_rad") ==
          doctest::Approx(expected["right_joint_angle_rad"]).epsilon(0.001));
  REQUIRE(response_payload.at("left_max_surface_angle_rad") ==
          doctest::Approx(expected["left_max_surface_angle_rad"]).epsilon(0.001));
  REQUIRE(response_payload.at("right_max_surface_angle_rad") ==
          doctest::Approx(expected["right_max_surface_angle_rad"]).epsilon(0.001));
}

// NOLINTEND(*-magic-numbers, readability-function-cognitive-complexity)
