#include <cstdint>

#include "common/messages/management.h"
#include "common/messages/weld_system.h"
#include "helpers/helpers_event_handling.h"
#include "helpers/helpers_joint_geometry.h"
#include "helpers/helpers_kinematics.h"
#include "helpers/helpers_settings.h"
#include "helpers/helpers_weld_system.h"
#include "tracking/tracking_manager.h"
#include "weld_system_client/weld_system_types.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access)

#include <doctest/doctest.h>

#include "common/messages/kinematics.h"
#include "common/messages/scanner.h"
#include "helpers/helpers.h"

namespace {
const float HORIZONTAL_OFFSET = 10.0;
const float VERTICAL_OFFSET   = 20.0;
}  // namespace

TEST_SUITE("Joint_tracking") {
  TEST_CASE("basic_sequence") {
    TestFixture fixture;
    fixture.StartApplication();

    StoreSettings(fixture, TestSettings{.use_edge_sensor = false}, true);
    StoreDefaultJointGeometryParams(fixture);

    // Start Joint tracking
    common::msg::management::TrackingStart start_joint_tracking_msg{
        static_cast<uint32_t>(tracking::TrackingMode::TRACKING_LEFT_HEIGHT), HORIZONTAL_OFFSET, VERTICAL_OFFSET};
    fixture.Management()->Dispatch(start_joint_tracking_msg);

    // Receive StartScanner
    CHECK(fixture.Scanner()->Receive<common::msg::scanner::Start>());

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
}

// NOLINTEND(*-magic-numbers, *-optional-access)
