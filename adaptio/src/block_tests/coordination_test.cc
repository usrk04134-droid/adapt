#include <cstdint>
#include <utility>

#include "common/messages/management.h"
#include "common/messages/weld_system.h"
#include "coordination/activity_status.h"
#include "helpers/helpers.h"
#include "helpers/helpers_joint_geometry.h"
#include "helpers/helpers_kinematics.h"
#include "helpers/helpers_settings.h"
#include "helpers/helpers_web_hmi.h"
#include "helpers/helpers_weld_system.h"
#include "tracking/tracking_manager.h"
#include "weld_system_client//weld_system_types.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access)

#include <doctest/doctest.h>

#include "common/messages/kinematics.h"
#include "common/messages/scanner.h"
#include "web_hmi/web_hmi_json_helpers.h"

const float HORIZONTAL_OFFSET       = 10.0;
const float VERTICAL_OFFSET         = 20.0;
const auto ACTIVITY_STATUS_IDLE     = static_cast<uint32_t>(coordination::ActivityStatusE::IDLE);
const auto ACTIVITY_STATUS_TRACKING = static_cast<uint32_t>(coordination::ActivityStatusE::TRACKING);

TEST_SUITE("Coordination") {
  TEST_CASE("Activity_status") {
    TestFixture fixture;
    fixture.StartApplication();

    StoreSettings(fixture, TestSettings{.use_edge_sensor = false}, true);
    StoreDefaultJointGeometryParams(fixture);

    // Check that status is IDLE
    {
      auto get_activity_status = web_hmi::CreateMessage("GetActivityStatus", {});
      fixture.WebHmiIn()->DispatchMessage(std::move(get_activity_status));
      auto status_payload = ReceiveJsonByName(fixture, "GetActivityStatusRsp");
      CHECK(status_payload != nullptr);
      CHECK_EQ(status_payload["value"], ACTIVITY_STATUS_IDLE);
    }

    // Start Joint tracking
    common::msg::management::TrackingStart start_joint_tracking_msg{
        static_cast<uint32_t>(tracking::TrackingMode::TRACKING_LEFT_HEIGHT), HORIZONTAL_OFFSET, VERTICAL_OFFSET};
    fixture.Management()->Dispatch(start_joint_tracking_msg);

    // Receive StartScanner
    CHECK(fixture.Scanner()->Receive<common::msg::scanner::Start>());

    // Check that status is TRACKING
    {
      auto get_activity_status = web_hmi::CreateMessage("GetActivityStatus", {});
      fixture.WebHmiIn()->DispatchMessage(std::move(get_activity_status));
      auto status_payload = ReceiveJsonByName(fixture, "GetActivityStatusRsp");
      CHECK(status_payload != nullptr);
      CHECK_EQ(status_payload["value"], ACTIVITY_STATUS_TRACKING);
    }

    // ABW points on scanner interface
    fixture.Scanner()->Dispatch(fixture.ScannerData()->Get());

    // Receive GetPosition
    auto get_position = fixture.Kinematics()->Receive<common::msg::kinematics::GetSlidesPosition>();
    CHECK(get_position);

    // GetPosition response
    fixture.Kinematics()->Dispatch(common::msg::kinematics::GetSlidesPositionRsp{.client_id  = get_position->client_id,
                                                                                 .time_stamp = get_position->time_stamp,
                                                                                 .horizontal = -20,
                                                                                 .vertical   = 5});

    CheckAndDispatchGetWeldAxis(fixture, 1.23, 0.0, 2.55, 3500);
    const common::msg::weld_system::GetWeldSystemDataRsp weld_system_status_rsp{
        .voltage           = 31.0,
        .current           = 216.123456,
        .wire_lin_velocity = 10.1,
        .deposition_rate   = 4.5,
        .heat_input        = 125.0,
        .twin_wire         = false,
        .wire_diameter     = 1.2,
    };

    CheckAndDispatchWeldSystemDataRsp(fixture, weld_system::WeldSystemId::ID1, weld_system_status_rsp);
    CheckAndDispatchWeldSystemDataRsp(fixture, weld_system::WeldSystemId::ID2, weld_system_status_rsp);

    // Finally receive SetPosition
    CHECK(fixture.Kinematics()->Receive<common::msg::kinematics::SetSlidesPosition>());
  }

  TEST_CASE("Double_start") {
    TestFixture fixture;
    fixture.StartApplication();

    StoreDefaultJointGeometryParams(fixture);
    // Start Joint tracking
    common::msg::management::TrackingStart start_joint_tracking_msg{
        static_cast<uint32_t>(tracking::TrackingMode::TRACKING_LEFT_HEIGHT), HORIZONTAL_OFFSET, VERTICAL_OFFSET};
    fixture.Management()->Dispatch(start_joint_tracking_msg);

    auto start_cal = web_hmi::CreateMessage("LaserToTorchCalibration", {});
    fixture.WebHmiIn()->DispatchMessage(std::move(start_cal));

    // Check that status is still TRACKING
    {
      auto get_activity_status = web_hmi::CreateMessage("GetActivityStatus", {});
      fixture.WebHmiIn()->DispatchMessage(std::move(get_activity_status));
      auto status_payload = ReceiveJsonByName(fixture, "GetActivityStatusRsp");
      CHECK(status_payload != nullptr);
      CHECK_EQ(status_payload["value"], ACTIVITY_STATUS_TRACKING);
    }
  }
}

// NOLINTEND(*-magic-numbers, *-optional-access)
