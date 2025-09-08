#pragma once

#include <doctest/doctest.h>

#include <optional>

#include "block_tests/helpers.h"
#include "helpers_web_hmi.h"

std::string const WELD_CONTROL_DATA_REQ  = "GetWeldControlStatus";
std::string const WELD_CONTROL_DATA_RSP  = "GetWeldControlStatusRsp";
std::string const WELD_CONTROL_START_ABP = "StartABP";

struct WeldControlStatus {
  std::optional<std::string> weld_control_mode;
  std::optional<std::string> bead_operation;
  std::optional<double> progress;
  std::optional<int> layer_number;
  std::optional<int> bead_number;
  std::optional<int> total_beads;
};

// NOLINTBEGIN(readability-function-cognitive-complexity)

inline auto CheckWeldControlStatus(TestFixture& fixture, const std::string& expect_weld_control_mode) {
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
  CHECK_EQ(weld_control_mode, expect_weld_control_mode);
  CHECK_EQ(bead_operation, "idle");
  REQUIRE_EQ(progress, doctest::Approx(0.));
  CHECK_EQ(bead_number, 0);
  CHECK_EQ(layer_number, 0);
  CHECK_FALSE(response_payload.contains("totalBeads"));
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
// NOLINTEND(readability-function-cognitive-complexity)
