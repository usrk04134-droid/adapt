#include <nlohmann/json_fwd.hpp>
#include <utility>
#include <vector>

#include "block_tests/helpers_joint_geometry.h"
#include "block_tests/helpers_web_hmi.h"
#include "common/messages/management.h"
#include "macs/macs_point.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access)

#include <doctest/doctest.h>

#include "common/messages/kinematics.h"
#include "helpers.h"
#include "web_hmi/web_hmi_json_helpers.h"

auto GrooveFromPayload(const nlohmann::json& payload) -> std::vector<macs::Point> {
  std::vector<macs::Point> groove;

  if (!payload.contains("groove")) {
    return groove;
  }

  auto payload_groove = payload.at("groove");

  for (auto& item : payload_groove) {
    macs::Point coordinate{};
    item.at("horizontal").get_to(coordinate.horizontal);
    item.at("vertical").get_to(coordinate.vertical);

    groove.push_back(coordinate);
  }

  return groove;
}

const float HORIZONTAL_SHIFT = 10.0;

TEST_SUITE("Coordinate_systems") {
  // The purpose of this test is to get a first check that the x axis of MACS is
  // reversed compared to LPCS
  // Shifting the LPCS data in the positive horizonal directon should
  // shift the MACS data in the negative horizontal direction.
  TEST_CASE("Orientation") {
    TestFixture fixture;
    fixture.StartApplication();

    StoreDefaultJointGeometryParams(fixture);
    // Start Joint tracking (to have the scanner in starte)
    common::msg::management::TrackingStart start_joint_tracking_msg{};
    fixture.Management()->Dispatch(start_joint_tracking_msg);

    // Dispatch scannerdata (LPCS)
    fixture.Scanner()->Dispatch(fixture.ScannerData()->Get());

    // Receive GetPosition
    auto get_position = fixture.Kinematics()->Receive<common::msg::kinematics::GetSlidesPosition>();
    CHECK(get_position);

    // GetPosition response
    fixture.Kinematics()->Dispatch(common::msg::kinematics::GetSlidesPositionRsp{
        .client_id = get_position->client_id, .time_stamp = get_position->time_stamp, .horizontal = 0, .vertical = 0});

    // Get Groove data (MACS)
    double groove_point_0_horizontal{};
    {
      auto get_groove = web_hmi::CreateMessage("GetGroove", {});
      fixture.WebHmiIn()->DispatchMessage(std::move(get_groove));

      auto groove_payload = ReceiveJsonByName(fixture, "GetGrooveRsp");
      auto groove         = GrooveFromPayload(groove_payload);
      CHECK(groove.size() > 0);
      groove_point_0_horizontal = groove[0].horizontal;
    }

    // Shift scannerdata and dispatch
    fixture.Scanner()->Dispatch(fixture.ScannerData()->ShiftHorizontal(HORIZONTAL_SHIFT).Get());

    // Receive SetPosition (not interesting in this testcase)
    fixture.Kinematics()->Receive<common::msg::kinematics::SetSlidesPosition>();

    // Receive GetPosition
    get_position = fixture.Kinematics()->Receive<common::msg::kinematics::GetSlidesPosition>();
    CHECK(get_position);

    // GetPosition response
    fixture.Kinematics()->Dispatch(common::msg::kinematics::GetSlidesPositionRsp{
        .client_id = get_position->client_id, .time_stamp = get_position->time_stamp, .horizontal = 0, .vertical = 0});

    // Get Groove data (MACS)
    double groove_point_0_horizontal_shifted{};
    {
      auto get_groove = web_hmi::CreateMessage("GetGroove", {});
      fixture.WebHmiIn()->DispatchMessage(std::move(get_groove));

      auto groove_payload = ReceiveJsonByName(fixture, "GetGrooveRsp");
      auto groove         = GrooveFromPayload(groove_payload);
      CHECK(groove.size() > 0);
      groove_point_0_horizontal_shifted = groove[0].horizontal;
    }

    CHECK(groove_point_0_horizontal_shifted < groove_point_0_horizontal);
  }
}
// NOLINTEND(*-magic-numbers, *-optional-access)
