#pragma once

#include <doctest/doctest.h>

#include "block_tests/helpers_web_hmi.h"
#include "helpers.h"
#include "helpers_json_compare.h"
#include "web_hmi/web_hmi_json_helpers.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access, hicpp-signed-bitwise)

inline const std::string WPP_ADD     = "AddWeldProcessParameters";
inline const std::string WPP_ADD_RSP = "AddWeldProcessParametersRsp";
inline const std::string WPP_GET     = "GetWeldProcessParameters";
inline const std::string WPP_GET_RSP = "GetWeldProcessParametersRsp";

inline auto AddWeldProcessParameters(TestFixture& fixture, nlohmann::json const& payload, bool expect_ok) {
  auto msg = web_hmi::CreateMessage(WPP_ADD, payload);
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));

  auto const response_payload = ReceiveJsonByName(fixture, WPP_ADD_RSP);
  CHECK(response_payload != nullptr);

  auto const expected = expect_ok
    ? nlohmann::json { { "result", "ok"} }
    : nlohmann::json { { "result", "fail"} };

  CHECK_EQ(response_payload, expected);
}

inline auto CheckWeldProcessParametersEqual(TestFixture& fixture, nlohmann::json const& expected) -> bool {
  auto msg = web_hmi::CreateMessage(WPP_GET, {});
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));

  auto const response_payload = ReceiveJsonByName(fixture, WPP_GET_RSP);

  return response_payload != nullptr && JsonEqualWithTolerance(response_payload, expected);
}

// NOLINTEND(*-magic-numbers, *-optional-access, hicpp-signed-bitwise)
