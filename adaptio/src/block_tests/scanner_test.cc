#include "common/messages/scanner.h"

#include <doctest/doctest.h>

#include <cstdint>

#include "block_tests/helpers_event_handling.h"
#include "block_tests/helpers_joint_geometry.h"
#include "common/messages/management.h"
#include "helpers.h"
#include "tracking/tracking_manager.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access)

namespace {
const float HORIZONTAL_OFFSET = 10.0;
const float VERTICAL_OFFSET   = 20.0;
}  // namespace

TEST_SUITE("scanner_test") {
  TEST_CASE("scanner_error") {
    TestFixture fixture;
    fixture.StartApplication();

    /* check that there are no events */
    CheckEvents(fixture, {});

    StoreDefaultJointGeometryParams(fixture);

    // Start Joint tracking
    common::msg::management::TrackingStart start_joint_tracking_msg{
        static_cast<uint32_t>(tracking::TrackingMode::TRACKING_LEFT_HEIGHT), HORIZONTAL_OFFSET, VERTICAL_OFFSET};
    fixture.Management()->Dispatch(start_joint_tracking_msg);

    // Receive StartScanner
    CHECK(fixture.Scanner()->Receive<common::msg::scanner::Start>());

    fixture.Scanner()->Dispatch(common::msg::scanner::StartRsp{.success = false});

    // Receive ScannerError
    CHECK(fixture.Management()->Receive<common::msg::management::ScannerError>());
  }
}

// NOLINTEND(*-magic-numbers, *-optional-access)
