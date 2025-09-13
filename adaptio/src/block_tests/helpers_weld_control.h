#pragma once

#include <doctest/doctest.h>

#include <optional>

#include "block_tests/helpers.h"
#include "helpers_web_hmi.h"

std::string const WELD_CONTROL_DATA_REQ          = "GetWeldControlStatus";
std::string const WELD_CONTROL_DATA_RSP          = "GetWeldControlStatusRsp";
std::string const WELD_CONTROL_START_ABP         = "StartABP";
std::string const WELD_CONTROL_CLEAR_SESSION     = "ClearWeldSession";
std::string const WELD_CONTROL_CLEAR_SESSION_RSP = "ClearWeldSessionRsp";

struct WeldControlStatus {
  std::optional<std::string> weld_control_mode;
  std::optional<std::string> bead_operation;
  std::optional<double> progress;
  std::optional<int> layer_number;
  std::optional<int> bead_number;
  std::optional<int> total_beads;
};

// NOLINTBEGIN(readability-function-cognitive-complexity)

inline auto CheckWeldControlStatus(TestFixture& fixture, const WeldControlStatus& expected) {
  auto msg = web_hmi::CreateMessage(WELD_CONTROL_DATA_REQ, nlohmann::json{});
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));

  auto response_payload = ReceiveJsonByName(fixture, WELD_CONTROL_DATA_RSP);
  CHECK(response_payload != nullptr);

  LOG_TRACE("WeldControl: {}", response_payload.dump());

  auto const weld_control_mode = response_payload.at("mode");
  auto const bead_operation    = response_payload.at("beadOperation");
  auto const progress          = response_payload.at("progress").get<double>();
  auto const bead_number       = response_payload.at("beadNumber").get<int>();
  auto const layer_number      = response_payload.at("layerNumber").get<int>();

  if (expected.weld_control_mode) {
    CHECK_EQ(weld_control_mode, expected.weld_control_mode);
  }

  if (expected.bead_operation) {
    CHECK_EQ(bead_operation, expected.bead_operation);
  }

  if (expected.progress) {
    REQUIRE_EQ(progress, doctest::Approx(*expected.progress));
  }

  if (expected.bead_number) {
    CHECK_EQ(bead_number, expected.bead_number);
  }

  if (expected.layer_number) {
    CHECK_EQ(layer_number, expected.layer_number);
  }

  if (expected.total_beads) {
    CHECK(response_payload.contains("totalBeads"));
    CHECK_EQ(expected.total_beads, response_payload.at("totalBeads").get<int>());
  }
}

inline auto GetWeldControlStatus(TestFixture& fixture) -> WeldControlStatus {
  auto msg = web_hmi::CreateMessage(WELD_CONTROL_DATA_REQ, nlohmann::json{});
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));

  auto response_payload = ReceiveJsonByName(fixture, WELD_CONTROL_DATA_RSP);
  CHECK(response_payload != nullptr);

  LOG_TRACE("WeldControl: {}", response_payload.dump());

  WeldControlStatus weld_control_status;

  if (response_payload.contains("mode")) {
    weld_control_status.weld_control_mode = response_payload.at("mode");
  }
  if (response_payload.contains("beadOperation")) {
    weld_control_status.bead_operation = response_payload.at("beadOperation");
  }
  if (response_payload.contains("progress")) {
    weld_control_status.progress = response_payload.at("progress");
  }
  if (response_payload.contains("layerNumber")) {
    weld_control_status.layer_number = response_payload.at("layerNumber");
  }
  if (response_payload.contains("beadNumber")) {
    weld_control_status.bead_number = response_payload.at("beadNumber");
  }
  if (response_payload.contains("totalBeads")) {
    weld_control_status.total_beads = response_payload.at("totalBeads");
  }
  return weld_control_status;
}

inline auto StartABP(TestFixture& fixture) { fixture.Management()->Dispatch(common::msg::management::ABPStart{}); }
inline auto StartABPCap(TestFixture& fixture) {
  fixture.Management()->Dispatch(common::msg::management::ABPCapStart{});
}

inline auto ClearWeldSession(TestFixture& fixture) -> void {
  auto msg = web_hmi::CreateMessage(WELD_CONTROL_CLEAR_SESSION, nlohmann::json{});
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));
}

inline auto CheckClearWeldSessionResponse(TestFixture& fixture, bool expect_success = true) -> void {
  auto response_payload = ReceiveJsonByName(fixture, WELD_CONTROL_CLEAR_SESSION_RSP);
  CHECK(response_payload != nullptr);

  LOG_TRACE("ClearWeldSessionRsp: {}", response_payload.dump());

  CHECK(response_payload.contains("result"));

  if (expect_success) {
    CHECK_EQ(response_payload.at("result"), "ok");
  } else {
    CHECK_EQ(response_payload.at("result"), "fail");
    CHECK(response_payload.contains("message"));
  }
}

inline auto ClearWeldSessionAndCheckResponse(TestFixture& fixture, bool expect_success = true) -> void {
  ClearWeldSession(fixture);
  CheckClearWeldSessionResponse(fixture, expect_success);
}

// NOLINTEND(readability-function-cognitive-complexity)
