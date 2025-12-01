#include <doctest/doctest.h>

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
    JointTracking(mfx, *simulator);

    // Check that status is TRACKING after starting joint tracking
    {
      auto get_activity_status = web_hmi::CreateMessage("GetActivityStatus", std::nullopt, {});
      mfx.Main().WebHmiIn()->DispatchMessage(std::move(get_activity_status));
      auto status_payload = ReceiveJsonByName(mfx.Main(), "GetActivityStatusRsp");
      CHECK(status_payload != nullptr);
      CHECK_EQ(status_payload.at("payload").at("value"), ACTIVITY_STATUS_TRACKING);
    }
  }
}

// NOLINTEND(*-magic-numbers, *-optional-access)
