#include <algorithm>
#include <cmath>
#include <doctest/doctest.h>
#include <iterator>

#include "block_tests/helpers/helpers_tracking.h"
#include "block_tests/helpers/helpers_web_hmi.h"
#include "coordination/activity_status.h"
#include "helpers/helpers.h"
#include "helpers/helpers_joint_geometry.h"
#include "helpers/helpers_settings.h"
#include "helpers/helpers_simulator.h"
#include "simulator_interface.h"
#include "web_hmi/web_hmi_json_helpers.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access)

namespace {
const int SIM_3D_OBJECT_SLICES_PER_REV{800};

const double WELD_OBJECT_DIAMETER_M = 2.0;
const double STICKOUT_M             = 25e-3;
const double WIRE_DIAMETER_MM       = 4.0;
const double WIRE_VELOCITY_MM_PER_SEC = 23.0;
const double SCANNER_MOUNT_ANGLE    = 6.0 * helpers_simulator::PI / 180.0;

const auto ACTIVITY_STATUS_IDLE     = static_cast<uint32_t>(coordination::ActivityStatusE::IDLE);
const auto ACTIVITY_STATUS_TRACKING = static_cast<uint32_t>(coordination::ActivityStatusE::TRACKING);
}  // namespace

TEST_SUITE("Joint_tracking") {
  TEST_CASE("joint_tracking_with_multi_fixture") {
    MultiFixture mfx;

    auto simulator  = deposition_simulator::CreateSimulator();
    auto sim_config = simulator->CreateSimConfig();
    helpers_simulator::SetSimulatorDefault(sim_config, SIM_3D_OBJECT_SLICES_PER_REV);

    helpers_simulator::SetJointGeometry(mfx.Main(), sim_config, helpers_simulator::TEST_JOINT_GEOMETRY_WIDE);
    StoreSettings(mfx.Main(), TestSettings{.use_edge_sensor = false}, true);

    auto depsim_ws1_torch = simulator->AddSingleWireTorch(
        helpers_simulator::ConvertMm2M(WIRE_DIAMETER_MM),
        helpers_simulator::ConvertMmPerS2MPerS(WIRE_VELOCITY_MM_PER_SEC));

    helpers_simulator::ConfigOPCS(sim_config, WELD_OBJECT_DIAMETER_M, STICKOUT_M);
    helpers_simulator::ConfigLPCS(sim_config, STICKOUT_M, SCANNER_MOUNT_ANGLE);

    simulator->Initialize(sim_config);

    // Check that status is IDLE before starting tracking
    {
      auto get_activity_status = web_hmi::CreateMessage("GetActivityStatus", std::nullopt, {});
      mfx.Main().WebHmiIn()->DispatchMessage(std::move(get_activity_status));
      auto status_payload = ReceiveJsonByName(mfx.Main(), "GetActivityStatusRsp");
      CHECK(status_payload != nullptr);
      CHECK_EQ(status_payload.at("payload").at("value"), ACTIVITY_STATUS_IDLE);
    }

    // Perform joint tracking
    const float jt_horizontal_offset = 0.0;
    const float jt_vertical_offset   = static_cast<float>(STICKOUT_M * 1000 + 1.0);
    JointTracking(mfx, *simulator, jt_horizontal_offset, jt_vertical_offset);

    // Check that status is TRACKING after starting joint tracking
    {
      auto get_activity_status = web_hmi::CreateMessage("GetActivityStatus", std::nullopt, {});
      mfx.Main().WebHmiIn()->DispatchMessage(std::move(get_activity_status));
      auto status_payload = ReceiveJsonByName(mfx.Main(), "GetActivityStatusRsp");
      CHECK(status_payload != nullptr);
      CHECK_EQ(status_payload.at("payload").at("value"), ACTIVITY_STATUS_TRACKING);
    }

    // Calculate expected position from the groove
    auto abw_in_torch_plane =
        helpers_simulator::ConvertFromOptionalAbwVector(simulator->GetSliceInTorchPlane(deposition_simulator::MACS));
    REQUIRE(abw_in_torch_plane.size() >= 7);  // Ensure we have ABW0-ABW6

    // For center tracking, the torch is positioned at ABW3's horizontal position plus offset
    const auto& abw3 = abw_in_torch_plane[3];
    const double torch_horizontal_m = abw3.GetX() + helpers_simulator::ConvertMm2M(jt_horizontal_offset);
    const double expected_horizontal_m = torch_horizontal_m;

    // Interpolate Z coordinate at the exact horizontal position of the torch
    // Find the two ABW points that bracket the torch horizontal position
    double interpolated_z = abw3.GetZ();  // Default to ABW3's Z
    if (torch_horizontal_m < abw3.GetX() && abw_in_torch_plane.size() > 3) {
      // Interpolate between ABW2 and ABW3
      const auto& abw2 = abw_in_torch_plane[2];
      if (abw2.GetX() != abw3.GetX()) {
        const double t = (torch_horizontal_m - abw2.GetX()) / (abw3.GetX() - abw2.GetX());
        interpolated_z = abw2.GetZ() + t * (abw3.GetZ() - abw2.GetZ());
      }
    } else if (torch_horizontal_m > abw3.GetX() && abw_in_torch_plane.size() > 4) {
      // Interpolate between ABW3 and ABW4
      const auto& abw4 = abw_in_torch_plane[4];
      if (abw4.GetX() != abw3.GetX()) {
        const double t = (torch_horizontal_m - abw3.GetX()) / (abw4.GetX() - abw3.GetX());
        interpolated_z = abw3.GetZ() + t * (abw4.GetZ() - abw3.GetZ());
      }
    }
    const double expected_vertical_m = interpolated_z + helpers_simulator::ConvertMm2M(jt_vertical_offset);

    // Check final torch position
    auto final_torch_pos = simulator->GetTorchPosition(deposition_simulator::MACS);
    const double tolerance_m = 0.001;  // 1mm tolerance
    CHECK(std::abs(final_torch_pos.GetX() - expected_horizontal_m) < tolerance_m);
    CHECK(std::abs(final_torch_pos.GetZ() - expected_vertical_m) < tolerance_m);
  }
}

// NOLINTEND(*-magic-numbers, *-optional-access)
