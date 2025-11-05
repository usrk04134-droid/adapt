#include <doctest/doctest.h>
#include <fmt/core.h>
#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <opencv2/imgcodecs.hpp>

#include "common/messages/kinematics.h"
#include "common/messages/management.h"
#include "common/messages/scanner.h"
#include "common/messages/weld_system.h"
#include "block_tests/helpers.h"
#include "block_tests/helpers_calibration_v2.h"
#include "block_tests/helpers_joint_geometry.h"
#include "block_tests/helpers_kinematics.h"
#include "block_tests/helpers_simulator.h"
#include "block_tests/helpers_weld_system.h"
#include "block_tests/helpers_web_hmi.h"
#include "point3d.h"
#include "scanner/image/image_builder.h"
#include "scanner/joint_model/snake.h"
#include "test_utils/testlog.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access)
// #define TESTLOG_DISABLED
#include "tracking/tracking_manager.h"
#include "weld_system_client/weld_system_types.h"

namespace depsim   = deposition_simulator;
namespace help_sim = helpers_simulator;

namespace {

// Calibration constants for image-based test
const double WELD_OBJECT_DIAMETER_M        = 4.0;  // 4 meter diameter based on typical pipe
const double STICKOUT_M                    = 25e-3;
const double WIRE_DIAMETER_MM              = 1.2;
const double SCANNER_MOUNT_ANGLE           = 0.26;  // radians (~15 degrees)

// Image test data path
const std::string TEST_IMAGE_PATH = "./tests/configs/sil/calibration/1738232679592.tiff";

/**
 * Extract groove coordinates from a scanner image
 * Returns SliceData with groove points extracted from the image
 */
auto ExtractGrooveFromImage(const std::string& image_path, uint64_t timestamp) 
    -> std::optional<common::msg::scanner::SliceData> {
  
  // Load the image
  auto grayscale_image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
  if (grayscale_image.empty()) {
    TESTLOG("Failed to load image: {}", image_path);
    return std::nullopt;
  }
  
  TESTLOG("Loaded image: {} ({}x{})", image_path, grayscale_image.cols, grayscale_image.rows);
  
  // Build scanner image
  auto maybe_image = scanner::image::ImageBuilder::From(grayscale_image, image_path, 0, 0).Finalize();
  if (!maybe_image.has_value()) {
    TESTLOG("Failed to build image from: {}", image_path);
    return std::nullopt;
  }
  
  auto* image = maybe_image.value().get();
  
  // Extract groove/snake from image
  auto snake_result = scanner::joint_model::Snake::FromImage(*image, {}, 16);
  if (!snake_result.has_value()) {
    TESTLOG("Failed to extract snake from image");
    return std::nullopt;
  }
  
  auto snake = snake_result.value();
  
  // Convert snake to SliceData format
  // The snake should have at least 7 points for the groove array
  if (snake.x.size() < common::msg::scanner::GROOVE_ARRAY_SIZE) {
    TESTLOG("Snake has insufficient points: {}", snake.x.size());
    return std::nullopt;
  }
  
  common::msg::scanner::SliceData slice_data;
  slice_data.time_stamp = timestamp;
  slice_data.confidence = common::msg::scanner::SliceConfidence::HIGH;
  slice_data.groove_area = 0.0;
  
  // Fill groove array with snake points
  // We sample evenly from the snake to get 7 representative points
  size_t step = snake.x.size() / (common::msg::scanner::GROOVE_ARRAY_SIZE - 1);
  for (size_t i = 0; i < common::msg::scanner::GROOVE_ARRAY_SIZE; ++i) {
    size_t idx = std::min(i * step, snake.x.size() - 1);
    slice_data.groove[i].x = snake.x[idx];
    slice_data.groove[i].y = snake.y[idx];
  }
  
  TESTLOG("Extracted groove with {} points from snake of size {}", 
          common::msg::scanner::GROOVE_ARRAY_SIZE, snake.x.size());
  
  return slice_data;
}

/**
 * Provide scanner data from image and simulated kinematics data
 */
void ProvideImageBasedScannerData(TestFixture& fixture, 
                                  const common::msg::scanner::SliceData& slice_data,
                                  double horizontal_mm, double vertical_mm) {
  // Dispatch the slice data from the image
  fixture.Scanner()->Dispatch(slice_data);
  
  // Receive GetSlidesPosition request
  auto get_position = fixture.Kinematics()->Receive<common::msg::kinematics::GetSlidesPosition>();
  if (!get_position.has_value()) {
    TESTLOG("Warning: No GetSlidesPosition request received");
    return;
  }
  
  // Respond with the provided position
  fixture.Kinematics()->Dispatch(
      common::msg::kinematics::GetSlidesPositionRsp{
          .client_id  = get_position->client_id,
          .time_stamp = get_position->time_stamp,
          .horizontal = horizontal_mm,
          .vertical   = vertical_mm
      });
}

/**
 * Perform calibration using real scanner image data
 */
auto CalibrateWithImage(TestFixture& fixture, const std::string& image_path) -> bool {
  uint64_t timestamp = fixture.GetClockNowFuncWrapper()->GetSystemClock().time_since_epoch().count();
  
  // Extract groove data from the image
  auto slice_data_opt = ExtractGrooveFromImage(image_path, timestamp);
  if (!slice_data_opt.has_value()) {
    TESTLOG("Failed to extract groove data from image");
    return false;
  }
  
  auto slice_data = slice_data_opt.value();
  
  // Calculate LTC parameters
  double ltc_stickout = help_sim::ConvertM2Mm(STICKOUT_M);
  double ltc_torch_to_laser_plane_dist = 150.0;  // mm, typical value
  
  // Subscribe to Ready State
  fixture.Management()->Dispatch(common::msg::management::SubscribeReadyState{});
  auto ready_msg = fixture.Management()->Receive<common::msg::management::ReadyState>();
  CHECK(ready_msg.has_value());
  CHECK_EQ(ready_msg->state, common::msg::management::ReadyState::State::NOT_READY);
  
  // Set laser to torch calibration
  LaserTorchCalSet(fixture, {
      {"distanceLaserTorch", ltc_torch_to_laser_plane_dist},
      {"stickout",           ltc_stickout                 },
      {"scannerMountAngle",  SCANNER_MOUNT_ANGLE          }
  });
  
  CHECK_EQ(LaserTorchCalSetRsp(fixture), nlohmann::json{{"result", "ok"}});
  
  // Start weld object calibration
  WeldObjectCalStart(fixture, WIRE_DIAMETER_MM, ltc_stickout,
                     help_sim::ConvertM2Mm(WELD_OBJECT_DIAMETER_M) / 2.0);
  
  // Receive and respond to StartScanner
  REQUIRE_MESSAGE(fixture.Scanner()->Receive<common::msg::scanner::Start>(), "No Start msg received");
  fixture.Scanner()->Dispatch(common::msg::scanner::StartRsp{.success = true});
  
  CHECK(WeldObjectCalStartRsp(fixture));
  
  // Calculate approximate touch positions based on groove points
  // For left touch: use leftmost point minus some offset for the wire
  double left_touch_horizontal = slice_data.groove[0].x - WIRE_DIAMETER_MM / 2.0;
  double left_touch_vertical = slice_data.groove[0].y;
  
  TESTLOG("Simulating left touch at horizontal: {:.2f}, vertical: {:.2f}", 
          left_touch_horizontal, left_touch_vertical);
  
  // Operator presses the left position button
  WeldObjectCalLeftPos(fixture);
  ProvideImageBasedScannerData(fixture, slice_data, left_touch_horizontal, left_touch_vertical);
  
  CHECK(WeldObjectCalLeftPosRsp(fixture));
  
  // For right touch: use rightmost point plus some offset for the wire
  double right_touch_horizontal = slice_data.groove[6].x + WIRE_DIAMETER_MM / 2.0;
  double right_touch_vertical = slice_data.groove[6].y;
  
  TESTLOG("Simulating right touch at horizontal: {:.2f}, vertical: {:.2f}", 
          right_touch_horizontal, right_touch_vertical);
  
  // Operator presses the right position button
  WeldObjectCalRightPos(fixture);
  ProvideImageBasedScannerData(fixture, slice_data, right_touch_horizontal, right_touch_vertical);
  
  // Check Ready state changed to NOT_READY_AUTO_CAL_MOVE
  ready_msg = fixture.Management()->Receive<common::msg::management::ReadyState>();
  CHECK(ready_msg);
  CHECK_EQ(ready_msg->state, common::msg::management::ReadyState::State::NOT_READY_AUTO_CAL_MOVE);
  
  CHECK(WeldObjectCalRightPosRsp(fixture));
  
  TESTLOG("Automatic grid measurement sequence started");
  
  // Process grid measurements
  int grid_point_count = 0;
  const int MAX_GRID_POINTS = 50;  // Safety limit
  
  while (grid_point_count < MAX_GRID_POINTS) {
    auto set_position = fixture.Kinematics()->Receive<common::msg::kinematics::SetSlidesPosition>();
    if (!set_position) {
      break;  // No more grid points requested
    }
    
    grid_point_count++;
    
    // Provide scanner data at the requested position
    ProvideImageBasedScannerData(fixture, slice_data, 
                                 set_position.value().horizontal,
                                 set_position.value().vertical);
    
    // Dispatch stabilization timer
    fixture.Timer()->Dispatch("stabilization_delay");
    
    // Provide scanner data again (for the actual measurement)
    ProvideImageBasedScannerData(fixture, slice_data,
                                 set_position.value().horizontal,
                                 set_position.value().vertical);
    
    // Receive progress update
    auto const payload = ReceiveJsonByName(fixture, "WeldObjectCalProgress");
    if (payload != nullptr) {
      auto progress = payload["progress"].get<double>();
      TESTLOG("Grid measurement #{}, progress: {:.1f}%", grid_point_count, progress);
    }
  }
  
  TESTLOG("Completed {} grid measurements", grid_point_count);
  
  // Get calibration result
  auto calibration_result_payload = ReceiveJsonByName(fixture, "WeldObjectCalResult");
  CHECK(calibration_result_payload != nullptr);
  
  if (calibration_result_payload["result"] == "ok") {
    TESTLOG("Calibration successful!");
    TESTLOG("Result: {}", calibration_result_payload.dump(2));
    
    // Check Ready state
    ready_msg = fixture.Management()->Receive<common::msg::management::ReadyState>();
    CHECK(ready_msg);
    CHECK_EQ(ready_msg->state, common::msg::management::ReadyState::State::NOT_READY);
    
    // Apply the calibration result
    WeldObjectCalSet(fixture, calibration_result_payload);
    auto set_rsp = WeldObjectCalSetRsp(fixture);
    CHECK_EQ(set_rsp, nlohmann::json{{"result", "ok"}});
    
    return true;
  } else {
    TESTLOG("Calibration failed: {}", calibration_result_payload["result"].get<std::string>());
    return false;
  }
}

}  // namespace

TEST_SUITE("WeldObjectCalibrationImage") {
  TEST_CASE("calibrate_from_real_image") {
    TestFixture fixture;
    
    // Start the application without default calibration
    fixture.Sut()->Start();
    fixture.SetupMockets();
    fixture.SetupTimerWrapper();
    
    // Set up joint geometry - use a standard configuration
    auto const payload = nlohmann::json({
        {"upper_joint_width_mm",        help_sim::TEST_JOINT_GEOMETRY_WIDE.upper_joint_width_mm       },
        {"groove_depth_mm",             help_sim::TEST_JOINT_GEOMETRY_WIDE.groove_depth_mm            },
        {"left_joint_angle_rad",        help_sim::TEST_JOINT_GEOMETRY_WIDE.left_joint_angle_rad       },
        {"right_joint_angle_rad",       help_sim::TEST_JOINT_GEOMETRY_WIDE.right_joint_angle_rad      },
        {"left_max_surface_angle_rad",  help_sim::TEST_JOINT_GEOMETRY_WIDE.left_max_surface_angle_rad },
        {"right_max_surface_angle_rad", help_sim::TEST_JOINT_GEOMETRY_WIDE.right_max_surface_angle_rad}
    });
    StoreJointGeometryParams(fixture, payload, true);
    
    // Set up kinematics state
    DispatchKinematicsStateChange(fixture, common::msg::kinematics::StateChange::State::HOMED);
    DispatchKinematicsEdgeStateChange(fixture, common::msg::kinematics::EdgeStateChange::State::AVAILABLE);
    
    // Perform calibration using the real image
    CHECK(CalibrateWithImage(fixture, TEST_IMAGE_PATH));
  }
  
  TEST_CASE("image_loading_test") {
    // Simple test to verify image can be loaded and processed
    auto grayscale_image = cv::imread(TEST_IMAGE_PATH, cv::IMREAD_GRAYSCALE);
    CHECK_FALSE(grayscale_image.empty());
    
    if (!grayscale_image.empty()) {
      TESTLOG("Image loaded successfully: {}x{}", grayscale_image.cols, grayscale_image.rows);
      
      auto maybe_image = scanner::image::ImageBuilder::From(grayscale_image, TEST_IMAGE_PATH, 0, 0).Finalize();
      CHECK(maybe_image.has_value());
      
      if (maybe_image.has_value()) {
        auto* image = maybe_image.value().get();
        auto snake_result = scanner::joint_model::Snake::FromImage(*image, {}, 16);
        CHECK(snake_result.has_value());
        
        if (snake_result.has_value()) {
          TESTLOG("Snake extracted with {} points", snake_result.value().x.size());
        }
      }
    }
  }
}

// NOLINTEND(*-magic-numbers, *-optional-access)
