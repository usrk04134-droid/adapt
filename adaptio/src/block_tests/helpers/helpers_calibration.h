#pragma once

#include <doctest/doctest.h>

#include <nlohmann/json_fwd.hpp>
#include <optional>

#include "common/messages/kinematics.h"
#include "helpers.h"
#include "helpers_mfx.h"
#include "helpers_simulator.h"
#include "helpers_web_hmi.h"
#include "simulator_interface.h"
#include "test_utils/testlog.h"
#include "web_hmi/web_hmi_json_helpers.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access, hicpp-signed-bitwise)
inline auto Merge(const nlohmann::json& payload1, const nlohmann::json& payload2) -> nlohmann::json {
  nlohmann::json result = payload1;
  result.update(payload2);
  return result;
}

[[nodiscard]] inline auto CheckResponseOk(TestFixture& fixture, const std::string& message_name) -> bool {
  auto const response_payload = ReceiveJsonByName(fixture, message_name);
  CHECK(response_payload != nullptr);

  return response_payload.at("result") == "ok";
}

inline void LaserTorchCalSet(TestFixture& fixture, const nlohmann::json& payload) {
  auto msg = web_hmi::CreateMessage("LaserTorchCalSet", std::nullopt, payload);
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));
}

[[nodiscard]] inline auto LaserTorchCalSetRsp(TestFixture& fixture) -> bool {
  return CheckResponseOk(fixture, "LaserTorchCalSetRsp");
}

inline void LaserTorchCalGet(TestFixture& fixture) {
  auto msg = web_hmi::CreateMessage("LaserTorchCalGet", std::nullopt, {});
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));
}

[[nodiscard]] inline auto LaserTorchCalGetRsp(TestFixture& fixture) -> nlohmann::json {
  return ReceiveJsonByName(fixture, "LaserTorchCalGetRsp");
}

inline void WeldObjectCalSet(TestFixture& fixture, const nlohmann::json& payload) {
  auto msg = web_hmi::CreateMessage("WeldObjectCalSet", std::nullopt, payload);
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));
}

[[nodiscard]] inline auto WeldObjectCalSetRsp(TestFixture& fixture) -> bool {
  return CheckResponseOk(fixture, "WeldObjectCalSetRsp");
}

inline void WeldObjectCalGet(TestFixture& fixture) {
  auto msg = web_hmi::CreateMessage("WeldObjectCalGet", std::nullopt, {});
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));
}

[[nodiscard]] inline auto WeldObjectCalGetRsp(TestFixture& fixture) -> nlohmann::json {
  return ReceiveJsonByName(fixture, "WeldObjectCalGetRsp");
}

inline void WeldObjectCalStart(TestFixture& fixture, double wire_diameter, double stickout, double weld_object_radius) {
  auto const payload = nlohmann::json({
      {"wireDiameter",     wire_diameter     },
      {"stickout",         stickout          },
      {"weldObjectRadius", weld_object_radius},
  });

  auto msg = web_hmi::CreateMessage("WeldObjectCalStart", std::nullopt, payload);
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));
}

[[nodiscard]] inline auto WeldObjectCalStartRsp(TestFixture& fixture) -> bool {
  return CheckResponseOk(fixture, "WeldObjectCalStartRsp");
}

inline void WeldObjectCalStop(TestFixture& fixture) {
  auto msg = web_hmi::CreateMessage("WeldObjectCalStop", std::nullopt, {});
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));
}

[[nodiscard]] inline auto WeldObjectCalStopRsp(TestFixture& fixture) -> bool {
  return CheckResponseOk(fixture, "WeldObjectCalStopRsp");
}

inline void WeldObjectCalTopPos(TestFixture& fixture) {
  auto msg = web_hmi::CreateMessage("WeldObjectCalTopPos", std::nullopt, {});
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));
}

[[nodiscard]] inline auto WeldObjectCalTopPosRsp(TestFixture& fixture) -> bool {
  return CheckResponseOk(fixture, "WeldObjectCalTopPosRsp");
}

inline void WeldObjectCalLeftPos(TestFixture& fixture) {
  auto msg = web_hmi::CreateMessage("WeldObjectCalLeftPos", std::nullopt, {});
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));
}

[[nodiscard]] inline auto WeldObjectCalLeftPosRsp(TestFixture& fixture) -> bool {
  return CheckResponseOk(fixture, "WeldObjectCalLeftPosRsp");
}

inline void WeldObjectCalRightPos(TestFixture& fixture) {
  auto msg = web_hmi::CreateMessage("WeldObjectCalRightPos", std::nullopt, {});
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));
}

[[nodiscard]] inline auto WeldObjectCalRightPosRsp(TestFixture& fixture) -> bool {
  return CheckResponseOk(fixture, "WeldObjectCalRightPosRsp");
}

inline auto WeldObjectCalResult(TestFixture& fixture) -> nlohmann::json {
  auto const response_payload = ReceiveJsonByName(fixture, "WeldObjectCalResult");
  CHECK(response_payload != nullptr);

  return response_payload;
}

inline void LWCalGet(TestFixture& fixture) {
  auto msg = web_hmi::CreateMessage("LWCalGet", std::nullopt, {});
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));
}

[[nodiscard]] inline auto LWCalGetRsp(TestFixture& fixture) -> nlohmann::json {
  return ReceiveJsonByName(fixture, "LWCalGetRsp");
}

inline void LWCalSet(TestFixture& fixture, const nlohmann::json& payload) {
  auto msg = web_hmi::CreateMessage("LWCalSet", std::nullopt, payload);
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));
}

[[nodiscard]] inline auto LWCalSetRsp(TestFixture& fixture) -> bool { return CheckResponseOk(fixture, "LWCalSetRsp"); }

inline void LWCalStart(TestFixture& fixture, double distance_laser_torch, double stickout, double scanner_mount_angle) {
  auto const payload = nlohmann::json({
      {"distanceLaserTorch", distance_laser_torch},
      {"stickout",           stickout            },
      {"scannerMountAngle",  scanner_mount_angle },
  });

  auto msg = web_hmi::CreateMessage("LWCalStart", std::nullopt, payload);
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));
}

[[nodiscard]] inline auto LWCalStartRsp(TestFixture& fixture) -> bool {
  return CheckResponseOk(fixture, "LWCalStartRsp");
}

inline void LWCalStop(TestFixture& fixture) {
  auto msg = web_hmi::CreateMessage("LWCalStop", std::nullopt, {});
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));
}

[[nodiscard]] inline auto LWCalStopRsp(TestFixture& fixture) -> bool {
  return CheckResponseOk(fixture, "LWCalStopRsp");
}

inline void LWCalTopPos(TestFixture& fixture) {
  auto msg = web_hmi::CreateMessage("LWCalTopPos", std::nullopt, {});
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));
}

[[nodiscard]] inline auto LWCalTopPosRsp(TestFixture& fixture) -> bool {
  return CheckResponseOk(fixture, "LWCalTopPosRsp");
}

inline void LWCalLeftPos(TestFixture& fixture) {
  auto msg = web_hmi::CreateMessage("LWCalLeftPos", std::nullopt, {});
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));
}

[[nodiscard]] inline auto LWCalLeftPosRsp(TestFixture& fixture) -> bool {
  return CheckResponseOk(fixture, "LWCalLeftPosRsp");
}

inline void LWCalRightPos(TestFixture& fixture) {
  auto msg = web_hmi::CreateMessage("LWCalRightPos", std::nullopt, {});
  fixture.WebHmiIn()->DispatchMessage(std::move(msg));
}

[[nodiscard]] inline auto LWCalRightPosRsp(TestFixture& fixture) -> bool {
  return CheckResponseOk(fixture, "LWCalRightPosRsp");
}

inline auto LWCalResult(TestFixture& fixture) -> nlohmann::json {
  auto const response_payload = ReceiveJsonByName(fixture, "LWCalResult");
  CHECK(response_payload != nullptr);

  return response_payload;
}

inline void ProvideScannerAndKinematicsData(TestFixture& fixture, deposition_simulator::ISimulator& simulator,
                                            const deposition_simulator::Point3d& point) {
  auto abws_lpcs  = helpers_simulator::ConvertFromOptionalAbwVector(simulator.GetAbwPoints(deposition_simulator::LPCS));
  auto slice_data = helpers_simulator::GetSliceData(abws_lpcs, simulator, NowTimeStamp(fixture));
  fixture.Scanner()->Dispatch(slice_data);

  // Receive GetSlidesPosition
  auto get_position = fixture.Kinematics()->Receive<common::msg::kinematics::GetSlidesPosition>();

  // GetSlidePosition response
  fixture.Kinematics()->Dispatch(
      common::msg::kinematics::GetSlidesPositionRsp{.client_id  = get_position->client_id,
                                                    .time_stamp = get_position->time_stamp,
                                                    .horizontal = helpers_simulator::ConvertM2Mm(point.GetX()),
                                                    .vertical   = helpers_simulator::ConvertM2Mm(point.GetZ())});
}

inline auto ReceiveProgress(TestFixture& fixture) -> double {
  auto const payload = ReceiveJsonByName(fixture, "WeldObjectCalProgress");
  CHECK(payload != nullptr);
  auto progress = payload.at("payload").at("progress").get<double>();
  CHECK(progress > 0);
  TESTLOG(">>>>> Weld Object calibration progress: {:.2f}", progress);
  return progress;
}

inline auto GridMeasurementAttempt(TestFixture& fixture, deposition_simulator::ISimulator& simulator) -> bool {
  auto set_position = fixture.Kinematics()->Receive<common::msg::kinematics::SetSlidesPosition>();
  if (!set_position) {
    return false;
  }

  auto horizontal_pos_m = helpers_simulator::ConvertMm2M(set_position.value().horizontal);
  auto vertical_pos_m   = helpers_simulator::ConvertMm2M(set_position.value().vertical);

  // Update the torch position according to the request
  deposition_simulator::Point3d torch_pos_macs(horizontal_pos_m, 0, vertical_pos_m, deposition_simulator::MACS);
  simulator.UpdateTorchPosition(torch_pos_macs);

  ProvideScannerAndKinematicsData(fixture, simulator, torch_pos_macs);

  // Dispatch timeout on stabilization timer
  fixture.Timer()->Dispatch("stabilization_delay");

  // Provide scannerdata again, this will be recorded (or skipped if an extra movement gridpoint)
  ProvideScannerAndKinematicsData(fixture, simulator, torch_pos_macs);

  ReceiveProgress(fixture);

  return true;
}

inline void PositionTorchInGroove(deposition_simulator::ISimulator& simulator, double stickout_m,
                                  double touch_point_depth_m) {
  // Position the torch with the tip of the wire at a depth TOUCH_POINT_DEPTH_M
  // in the center of the groove
  auto abw_in_torch_plane =
      helpers_simulator::ConvertFromOptionalAbwVector(simulator.GetSliceInTorchPlane(deposition_simulator::MACS));
  TESTLOG(">>>>> Torch plane ABW0: {}, ABW6: {}", ToString(abw_in_torch_plane.front()),
          ToString(abw_in_torch_plane.back()));

  deposition_simulator::Point3d torch_pos_initial_macs(
      std::midpoint(abw_in_torch_plane.front().GetX(), abw_in_torch_plane.back().GetX()), 0,
      abw_in_torch_plane.front().GetZ() + stickout_m - touch_point_depth_m, deposition_simulator::MACS);

  simulator.UpdateTorchPosition(torch_pos_initial_macs);
}

inline void PositionTorchAtTopLeftTouchPoint(deposition_simulator::ISimulator& simulator, double stickout_m,
                                             double abw0_horizontal_offset_m) {
  // Position the torch with the tip of the wire to touch the top with a given horizontal offset to abw0
  auto abw_in_torch_plane =
      helpers_simulator::ConvertFromOptionalAbwVector(simulator.GetSliceInTorchPlane(deposition_simulator::MACS));
  TESTLOG(">>>>> Torch plane ABW0: {}, ABW6: {}", ToString(abw_in_torch_plane.front()),
          ToString(abw_in_torch_plane.back()));

  deposition_simulator::Point3d torch_pos_initial_macs(abw_in_torch_plane.front().GetX() + abw0_horizontal_offset_m, 0,
                                                       abw_in_torch_plane.front().GetZ() + stickout_m,
                                                       deposition_simulator::MACS);

  simulator.UpdateTorchPosition(torch_pos_initial_macs);
}

// NOLINTEND(*-magic-numbers, *-optional-access, hicpp-signed-bitwise)
