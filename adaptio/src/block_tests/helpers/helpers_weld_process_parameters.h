#pragma once

#include <doctest/doctest.h>

#include <cstddef>

#include "helpers.h"
#include "helpers_json_compare.h"
#include "helpers_web_hmi.h"
#include "web_hmi/web_hmi_json_helpers.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access, hicpp-signed-bitwise)

inline const std::string WPP_ADD     = "AddWeldProcessParameters";
inline const std::string WPP_ADD_RSP = "AddWeldProcessParametersRsp";
inline const std::string WPP_GET     = "GetWeldProcessParameters";
inline const std::string WPP_GET_RSP = "GetWeldProcessParametersRsp";

inline auto AddWeldProcessParameters(TestFixture& fixture, nlohmann::json const& payload, bool expect_ok) {
  auto msg = web_hmi::CreateMessage(WPP_ADD, std::nullopt, payload);
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));

  auto const response_payload = ReceiveJsonByName(fixture, WPP_ADD_RSP);
  CHECK(response_payload != nullptr);

  auto const expected = expect_ok
    ? nlohmann::json { { "result", "ok"} }
    : nlohmann::json { { "result", "fail"} };

  CHECK_EQ(response_payload.at("result"), expected.at("result"));
}

inline auto CheckWeldProcessParametersEqual(TestFixture& fixture, nlohmann::json const& expected) -> bool {
  auto msg = web_hmi::CreateMessage(WPP_GET, std::nullopt, {});
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));

  auto const response_payload = ReceiveJsonByName(fixture, WPP_GET_RSP);
  CHECK(response_payload != nullptr);
  const auto& payload = response_payload.at("payload");

  return payload != nullptr && JsonEqualWithTolerance(payload, expected);
}

// NOLINTEND(*-magic-numbers, *-optional-access, hicpp-signed-bitwise)
