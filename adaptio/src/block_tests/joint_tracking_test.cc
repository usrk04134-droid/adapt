#include <cstdint>
#include <memory>

#include "block_tests/helpers/helpers_mfx_tracking.h"
#include "block_tests/helpers/helpers_web_hmi.h"
#include "common/messages/management.h"
#include "coordination/activity_status.h"
#include "helpers/helpers_joint_geometry.h"
#include "helpers/helpers_settings.h"
#include "helpers/helpers_simulator.h"
#include "simulator_interface.h"
#include "tracking/tracking_manager.h"
#include "web_hmi/web_hmi_json_helpers.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access)

#include <doctest/doctest.h>

#include "helpers/helpers.h"

namespace depsim   = deposition_simulator;
namespace help_sim = helpers_simulator;

namespace {
const int SIM_3D_OBJECT_SLICES_PER_REV{800};

const double WELD_OBJECT_DIAMETER_M = 2.0;
const double STICKOUT_M             = 25e-3;
// const double TOUCH_POINT_DEPTH_M      = 10e-3;
const double WIRE_DIAMETER_MM         = 4.0;
const double WIRE_VELOCITY_MM_PER_SEC = 23.0;
const double SCANNER_MOUNT_ANGLE      = 6.0 * help_sim::PI / 180.0;
}  // namespace

TEST_SUITE("Joint_tracking") {
  TEST_CASE("basic_sequence") {
    MultiFixture mfx;

    auto simulator  = depsim::CreateSimulator();
    auto sim_config = simulator->CreateSimConfig();
    help_sim::SetSimulatorDefault(sim_config, SIM_3D_OBJECT_SLICES_PER_REV);

    help_sim::SetJointGeometry(mfx.Main(), sim_config, help_sim::TEST_JOINT_GEOMETRY_WIDE);
    StoreSettings(mfx.Main(), TestSettings{.use_edge_sensor = false}, true);

    auto depsim_ws1_torch = simulator->AddSingleWireTorch(help_sim::ConvertMm2M(WIRE_DIAMETER_MM),
                                                          help_sim::ConvertMmPerS2MPerS(WIRE_VELOCITY_MM_PER_SEC));

    help_sim::ConfigOPCS(sim_config, WELD_OBJECT_DIAMETER_M, STICKOUT_M);
    help_sim::ConfigLPCS(sim_config, STICKOUT_M, SCANNER_MOUNT_ANGLE);

    simulator->Initialize(sim_config);

    // Check availability status from webhmi before starting tracking
    {
      auto get_availability_status = web_hmi::CreateMessage("GetActivityStatus", std::nullopt, {});
      mfx.Main().WebHmiIn()->DispatchMessage(std::move(get_availability_status));
      auto status_payload = ReceiveJsonByName(mfx.Main(), "GetActivityStatusRsp");
      CHECK(status_payload != nullptr);
      // Check that system is available (IDLE state means available for tracking)
      const auto ACTIVITY_STATUS_IDLE = static_cast<uint32_t>(coordination::ActivityStatusE::IDLE);
      CHECK_EQ(status_payload.at("payload").at("value"), ACTIVITY_STATUS_IDLE);
    }
    const float jt_horizontal_offset = 0.0;
    const float jt_vertical_offset   = static_cast<float>(STICKOUT_M * 1000 + 1.0);
    JointTracking(mfx, *simulator, jt_horizontal_offset, jt_vertical_offset);
    auto abw_in_torch_plane = help_sim::ConvertFromOptionalAbwVector(simulator->GetSliceInTorchPlane(depsim::MACS));
    auto expected_x         = std::midpoint(abw_in_torch_plane.front().GetX(), abw_in_torch_plane.back().GetX());
    auto expected_z         = abw_in_torch_plane[3].GetZ() + STICKOUT_M;

    auto final_torch_pos     = simulator->GetTorchPosition(depsim::MACS);
    const double tolerance_m = 0.001;
    CHECK(std::abs(final_torch_pos.GetX() - expected_x) < tolerance_m);
    CHECK(std::abs(final_torch_pos.GetZ() - expected_z) < tolerance_m);
  }
}

// NOLINTEND(*-magic-numbers, *-optional-access)