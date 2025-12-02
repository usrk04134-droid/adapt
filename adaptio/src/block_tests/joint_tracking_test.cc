#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <numeric>

#include "block_tests/helpers/helper_mfx_tracking.h"
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

const double WELD_OBJECT_DIAMETER_M   = 2.0;
const double STICKOUT_M               = 25e-3;
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

    JointTracking(mfx, *simulator);

    // Check that the torch is roughly at the correct position based on the groove geometry
    // For TRACKING_CENTER_HEIGHT, the torch should be at the groove center
    auto abw_in_torch_plane =
        help_sim::ConvertFromOptionalAbwVector(simulator->GetSliceInTorchPlane(depsim::MACS));
    auto expected_x = std::midpoint(abw_in_torch_plane.front().GetX(), abw_in_torch_plane.back().GetX());
    
    // Calculate groove center depth: find the deepest point (minimum Z, since Z is negative downward)
    auto min_z_it = std::min_element(abw_in_torch_plane.begin(), abw_in_torch_plane.end(),
                                      [](const auto& a, const auto& b) { return a.GetZ() < b.GetZ(); });
    auto max_z_it = std::max_element(abw_in_torch_plane.begin(), abw_in_torch_plane.end(),
                                     [](const auto& a, const auto& b) { return a.GetZ() < b.GetZ(); });
    // Groove center depth is the midpoint between top and bottom of groove
    auto expected_z = std::midpoint(max_z_it->GetZ(), min_z_it->GetZ());

    auto final_torch_pos = simulator->GetTorchPosition(depsim::MACS);
    const double tolerance_x_m = 0.001;  // 1mm tolerance for X
    const double tolerance_z_m = 0.01;    // 10mm tolerance for Z (tracking may position slightly differently)
    CHECK(std::abs(final_torch_pos.GetX() - expected_x) < tolerance_x_m);
    CHECK(std::abs(final_torch_pos.GetZ() - expected_z) < tolerance_z_m);
  }
}

// NOLINTEND(*-magic-numbers, *-optional-access)
