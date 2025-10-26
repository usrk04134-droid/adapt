#include "core/scanner/scanner_impl.h"

#include <prometheus/counter.h>
#include <prometheus/gauge.h>
#include <prometheus/histogram.h>
#include <prometheus/registry.h>

#include <algorithm>
#include <array>
#include <boost/asio/post.hpp>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <Eigen/Core>
#include <expected>
#include <iterator>
#include <memory>
#include <optional>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "core/image/camera_model.h"
#include "core/image/image.h"
#include "core/image/image_types.h"  // IWYU pragma: keep
#include "core/joint_tracking/joint_slice.h"
#include "core/logging/application_log.h"
#include "core/scanner/image_logger.h"
#include "core/scanner/image_provider.h"
#include "core/scanner/joint_buffer/joint_buffer.h"
#include "core/scanner/joint_model.h"
#include "core/scanner/scanner.h"
#include "core/scanner/scanner_types.h"

using core::image::CameraModelPtr;
using core::joint_tracking::SliceConfidence;
using core::scanner::JointProperties;
using core::scanner::ScannerErrorCode;
using core::scanner::ScannerImpl;

#define RECENT_SLICES_MS (400)
// #define DEBUG_RESULT_OUTPUT 1

#if defined(DEBUG_RESULT_OUTPUT)
#include <format>
#include <fstream>
#include <ios>
#include <ostream>
#endif

namespace {
const int WINDOW_MARGIN      = 100;
const int MOVE_MARGIN        = 40;
const int MINIMUM_FOV_HEIGHT = 500;
// Horizontal ROI tuning
const int H_WINDOW_MARGIN   = 200;   // pixels beyond ABW0/ABW6 on each side
const int H_MOVE_MARGIN     = 40;    // hysteresis to avoid chattering
const int MINIMUM_FOV_WIDTH = 1200;  // never crop narrower than this
}  // namespace

ScannerImpl::ScannerImpl(ImageProvider* image_provider, JointBufferPtr joint_buffer, LaserCallback laser_toggle,
                         ScannerOutputCB* scanner_output, JointModelPtr joint_model, ImageLogger* image_logger,
                         prometheus::Registry* registry)
    : image_provider_(image_provider),
      joint_model_(std::move(joint_model)),
      joint_buffer_(std::move(joint_buffer)),
      laser_toggle_(std::move(laser_toggle)),
      scanner_output_(scanner_output),
      image_logger_(image_logger),
      m_threadpool(12),
      store_image_data_(false) {
  SetupMetrics(registry);
}

void ScannerImpl::SetupMetrics(prometheus::Registry* registry) {
  {
    auto& counter = prometheus::BuildCounter()
                        .Name("scanner_image_process_success")
                        .Help("Number of processed successful images including the number of walls found.")
                        .Register(*registry);

    metrics_.image.emplace(0, &counter.Add({
                                  {"found", "0"}
    }));
    metrics_.image.emplace(1, &counter.Add({
                                  {"found", "1"}
    }));
    metrics_.image.emplace(2, &counter.Add({
                                  {"found", "2"}
    }));
  }

  {
    auto& counter = prometheus::BuildCounter()
                        .Name("scanner_image_process_errors")
                        .Help("Number of processed error images including the failure reason.")
                        .Register(*registry);

    auto const codes = {
        JointModelErrorCode::NO_ERROR,
        JointModelErrorCode::SURFACE_NOT_FOUND,
        JointModelErrorCode::WEDGE_FIT_FAILED,
        JointModelErrorCode::GROOVE_BOTTOM_NOT_FOUND,
        JointModelErrorCode::GROOVE_WALL_CENTROIDS_NOT_FOUND,
        JointModelErrorCode::MISSING_WEDGE_HISTORY,
        JointModelErrorCode::INVALID_SNAKE,
        JointModelErrorCode::INVALID_WALL_HEIGHT_DIFFERENCE,
        JointModelErrorCode::SURFACE_ANGLE_TOLERANCE_EXCEEDED,
        JointModelErrorCode::JOINT_WIDTH_OUT_OF_TOLERANCE,
        JointModelErrorCode::TWO_WALLS_NOT_FOUND,
        JointModelErrorCode::FAULTY_HISTORY_DATA,
    };

    for (auto code : codes) {
      metrics_.image_errors.emplace(code, &counter.Add({
                                              {"error", JointModelErrorCodeToSnakeCaseString(code)}
      }));
    }
  }

  {
    const std::vector<double> buckets = {
        0.005,  // 5 ms
        0.010,  // 10 ms
        0.020,  // 20 ms
        0.030,  // 30 ms
        0.040,  // 40 ms
        0.050,  // 50 ms
        0.060,  // 60 ms
        0.070,  // 70 ms
        0.080,  // 80 ms
        0.090,  // 90 ms
        0.100,  // 100 ms
        0.150,  // 150 ms
        0.200,  // 200 ms
    };

    auto& histogram = prometheus::BuildHistogram()
                          .Name("scanner_image_processing_duration_seconds")
                          .Help("Histogram of image processing durations for both successful and failed images.")
                          .Register(*registry);

    metrics_.image_processing_time = &histogram.Add({}, buckets);
  }

  {
    metrics_.image_consecutive_errors = &prometheus::BuildGauge()
                                             .Name("scanner_image_process_consecutive_errors")
                                             .Help("Number of consecutive error for processed.")
                                             .Register(*registry)
                                             .Add({});
  }
}

auto ScannerImpl::Start(enum ScannerSensitivity sensitivity) -> boost::outcome_v2::result<void> {
  LOG_TRACE("Starting Scanner");
  if (!joint_model_) {
    return ScannerErrorCode::NO_JOINT_PROPERTIES;
  }

  latest_sent   = std::chrono::high_resolution_clock::now();
  auto on_image = [this](std::unique_ptr<core::image::Image> img) -> void { ImageGrabbed(std::move(img)); };
  image_provider_->SetOnImage(on_image);
  return image_provider_->Start(sensitivity);
}

auto ScannerImpl::Start(enum ScannerSensitivity sensitivity, bool store_image_data) -> boost::outcome_v2::result<void> {
  store_image_data_ = store_image_data;
  return Start(sensitivity);
}

void ScannerImpl::Stop() {
  LOG_TRACE("Stopping Scanner");
  m_config_mutex.lock();
  image_provider_->Stop();
  m_config_mutex.unlock();

  m_threadpool.join();
  empty_joint_properties_ = {};
  image_provider_->SetOnImage(nullptr);
  dont_allow_fov_change_until_new_dimensions_received = std::nullopt;
}

void ScannerImpl::ClearJointBuffer() { joint_buffer_->Reset(); }

auto ScannerImpl::GetJointBuffer() -> const JointBuffer* { return joint_buffer_.get(); }

auto ScannerImpl::MedianOfRecentSlices() -> std::optional<core::scanner::JointSlice> {
  // WARNING: only the points, and the confidence is copied
  auto recent = joint_buffer_->GetRecentSlices(RECENT_SLICES_MS);

  if (recent.size() < 3) {
    return std::nullopt;
  }
  std::vector<double> x;
  std::transform(recent.begin(), recent.end(), std::back_inserter(x),
                 [](JointSlice* slice) { return slice->profile.points[0].x; });

  const auto middle = recent.size() / 2;
  double median_abw0_x;
  if (recent.size() & 1) {
    std::nth_element(x.begin(), x.begin() + middle, x.end());
    median_abw0_x = x[middle];
  } else {
    std::nth_element(x.begin(), x.begin() + middle, x.end());
    auto x1 = x[middle];
    std::nth_element(x.begin(), x.begin() + middle + 1, x.end());
    auto x2       = x[middle + 1];
    median_abw0_x = 0.5 * (x1 + x2);
  }
  core::scanner::JointSlice slice;

  int included = 0;
  std::vector<Timestamp> times;
  for (auto& old : recent) {
    if (fabs(old->profile.points[0].x - median_abw0_x) < 0.001) {
      included++;
      for (int i = 0; i < 7; i++) {
        slice.profile.points[i].x += old->profile.points[i].x;
        slice.profile.points[i].y += old->profile.points[i].y;
      }
      slice.profile.area    += old->profile.area;
      slice.num_walls_found += old->num_walls_found;
      times.push_back(old->timestamp);
    }
  }
  if (included == 0) {
    return std::nullopt;
  }
  for (int i = 0; i < 7; i++) {
    slice.profile.points[i].x /= included;
    slice.profile.points[i].y /= included;
  }
  slice.profile.area    /= included;
  slice.num_walls_found /= included;
  std::nth_element(times.begin(), times.begin() + included / 2, times.end());
  slice.timestamp = times[included / 2];

  return slice;
}

auto ScannerImpl::NewOffsetAndHeight(int top, int bottom) -> std::tuple<int, int> {
  auto new_offset = (top > WINDOW_MARGIN) ? (top - WINDOW_MARGIN) : 0;
  auto new_height = (bottom - new_offset) + WINDOW_MARGIN;

  if (new_height < MINIMUM_FOV_HEIGHT) {
    const auto adjust = (MINIMUM_FOV_HEIGHT - new_height) / 2;
    new_offset        = (new_offset > adjust) ? (new_offset - adjust) : 0;
    new_height        = MINIMUM_FOV_HEIGHT;
  }
  return {new_offset, new_height};
}

void ScannerImpl::ImageGrabbed(std::unique_ptr<core::image::Image> image) {
  boost::asio::post(m_threadpool, [this, image = std::move(image)]() {
    // When image capture is faster than parsing we need to be able to evaluate concurrently across multiple cores
    // This means that Parse should be a constant function. Any state should be recoverable from the latest slice.

    auto const start_timstamp = std::chrono::steady_clock::now();
    auto log_failed_image     = false;
    std::string reason_failed_image;

    m_buffer_mutex.lock();
    auto median_profile = MedianOfRecentSlices().transform(
        [](const core::scanner::JointSlice& slice) -> JointProfile { return slice.profile; });
    auto empty_joint_properties = empty_joint_properties_;
    m_buffer_mutex.unlock();

    auto result = joint_model_->Parse(*image.get(), median_profile, empty_joint_properties);

    num_received++;

    if (result) {
      auto [profile, centroids_wcs, processing_time, num_walls_found] = *result;
      LOG_TRACE("Processed image {} in {} ms.", image->GetImageName(), processing_time);
      JointSlice slice = {.uuid                = image->GetUuid(),
                          .timestamp           = image->GetTimestamp(),
                          .image_name          = image->GetImageName(),
                          .profile             = profile,
                          .num_walls_found     = num_walls_found,
                          .processing_time     = processing_time,
                          .vertical_crop_start = image->GetVerticalCropStart()};
      if (store_image_data_) {
        // Store image data only if necessary. Not needed when running Adaptio
        slice.image_data = image->Data();
        slice.centroids  = centroids_wcs;
      }

      m_buffer_mutex.lock();

      // If this is older than the last item in the buffer, don't add it.
      if (joint_buffer_->GetNumberOfSlices() == 0 || slice.timestamp > joint_buffer_->GetLatestTimestamp()) {
        joint_buffer_->AddSlice(slice);
#if defined(DEBUG_RESULT_OUTPUT)
        std::ofstream fp(std::format("results.txt", image->GetImageName()), std::ios::out | std::ios::app);
        fp << image->GetImageName() << "\t" << profile.points[0].x << "\t" << profile.points[0].y << "\t"
           << profile.points[1].x << "\t" << profile.points[1].y << "\t" << profile.points[2].x << "\t"
           << profile.points[2].y << "\t" << profile.points[3].x << "\t" << profile.points[3].y << "\t"
           << profile.points[4].x << "\t" << profile.points[4].y << "\t" << profile.points[5].x << "\t"
           << profile.points[5].y << "\t" << profile.points[6].x << "\t" << profile.points[6].y << "\t"
           << num_walls_found << "\t" << profile.area << " " << slice.vertical_crop_start << " ";

        auto points =
            joint_model_->WorkspaceToImage(ABWPointsToMatrix(profile.points), slice.vertical_crop_start).value();
        auto centroids = joint_model_->WorkspaceToImage(slice.centroids, slice.vertical_crop_start).value();
        fp << points.row(0) << " " << points.row(1) << " ";
        fp << centroids.row(0) << " " << centroids.row(1) << std::endl;
#endif
      } else {
        LOG_WARNING("Processed image is too old, discarding it.");
      }
      m_buffer_mutex.unlock();

      const int current_offset = image->GetVerticalCropStart();
      const int current_height = image->Data().rows();
      const auto [top, bottom] = profile.vertical_limits;
      m_config_mutex.lock();
      const auto dim_check = dont_allow_fov_change_until_new_dimensions_received;
      if (dim_check
              .transform([current_offset, current_height](std::tuple<int, int> requested) {
                auto [requested_offset, requested_height] = requested;
                return requested_offset == current_offset && requested_height == current_height;
              })
              .value_or(true)) {
        // Check min/max vertical pixels in the image and evaluate whether we want to adjust the FOV
        // Note: this assumes that the original offset is 0.
        // We want current_offset + WINDOW_MARGIN = top
        // and current_offset + height - WINDOW_MARGIN = bottom
        // If we are more than MOVE_MARGIN away from either of these two we recalculate

        const bool fov_is_small_but_covers_joint =
            current_height == MINIMUM_FOV_HEIGHT && bottom - top + 2 * WINDOW_MARGIN <= MINIMUM_FOV_HEIGHT &&
            current_offset + WINDOW_MARGIN <= top && bottom + WINDOW_MARGIN <= current_offset + current_height;

        if (!fov_is_small_but_covers_joint &&
            ((abs((top - WINDOW_MARGIN) - current_offset) > MOVE_MARGIN) ||
             (abs((bottom + WINDOW_MARGIN) - (current_offset + current_height)) > MOVE_MARGIN))) {
          auto [new_offset, new_height] = ScannerImpl::NewOffsetAndHeight(top, bottom);

          if (new_offset != current_offset || new_height != current_height) {
            LOG_TRACE(
                "Change FOV based on top {} bottom {}, current_offset {}, current_height {}, new_offset {} new_height "
                "{}",
                top, bottom, current_offset, current_height, new_offset, new_height);
            dont_allow_fov_change_until_new_dimensions_received = {new_offset, new_height};
            image_provider_->SetVerticalFOV(new_offset, new_height);
          }
        } else {
          dont_allow_fov_change_until_new_dimensions_received = std::nullopt;
        }
      }

      // Horizontal ROI adjustment based on ABW0/ABW6 (median if available)
      {
        auto points_to_use = median_profile.value_or(profile).points;
        auto maybe_img_pts =
            joint_model_->WorkspaceToImage(ABWPointsToMatrix(points_to_use), image->GetVerticalCropStart());
        if (maybe_img_pts) {
          const auto img_pts          = maybe_img_pts.value();
          const int abw0_x            = static_cast<int>(img_pts.row(0)[0]);
          const int abw6_x            = static_cast<int>(img_pts.row(0)[6]);
          const int desired_left_px   = std::max(0, std::min(abw0_x, abw6_x) - H_WINDOW_MARGIN);
          const int desired_right_px  = std::max(abw0_x, abw6_x) + H_WINDOW_MARGIN;
          int new_h_offset_from_left  = desired_left_px;
          int new_h_width             = std::max(MINIMUM_FOV_WIDTH, desired_right_px - desired_left_px);

          const int cur_h_offset_from_left = image_provider_->GetHorizontalFOVOffset();
          const int cur_h_width            = image_provider_->GetHorizontalFOVWidth();

          const bool width_diff  = std::abs(cur_h_width - new_h_width) > H_MOVE_MARGIN;
          const bool offset_diff = std::abs(cur_h_offset_from_left - new_h_offset_from_left) > H_MOVE_MARGIN;

          const auto hdim_check = dont_allow_hfov_change_until_new_dimensions_received;
          if (hdim_check
                  .transform([cur_h_offset_from_left, cur_h_width](std::tuple<int, int> requested) {
                    auto [req_off, req_w] = requested;
                    return req_off == cur_h_offset_from_left && req_w == cur_h_width;
                  })
                  .value_or(true)) {
            if (width_diff || offset_diff) {
              dont_allow_hfov_change_until_new_dimensions_received = {new_h_offset_from_left, new_h_width};
              image_provider_->SetHorizontalFOV(new_h_offset_from_left, new_h_width);
              LOG_TRACE(
                  "Change horizontal FOV based on ABW0 {} ABW6 {}, new_offset {} new_width {} (cur_off {} cur_w {})",
                  abw0_x, abw6_x, new_h_offset_from_left, new_h_width, cur_h_offset_from_left, cur_h_width);
            } else {
              dont_allow_hfov_change_until_new_dimensions_received = std::nullopt;
            }
          }
        }
      }

      if (++frames_since_gain_change_ > 100 && profile.suggested_gain_change.has_value()) {
        image_provider_->AdjustGain(profile.suggested_gain_change.value());
        frames_since_gain_change_ = 0;
      }
      m_config_mutex.unlock();

      if (metrics_.image.contains(slice.num_walls_found)) {
        metrics_.image.at(slice.num_walls_found)->Increment();
      }

      metrics_.image_consecutive_errors->Set(0);
    } else {
      auto const error = result.error();
      LOG_ERROR("Unable to parse joint in image {}: {}", image->GetImageName(), JointModelErrorCodeToString(error));

      if (!median_profile.has_value()) {
        m_config_mutex.lock();
        const auto dim_check = dont_allow_fov_change_until_new_dimensions_received;
        if (!dim_check.has_value() && image->GetVerticalCropStart() != 0 && frames_since_gain_change_ > 25) {
          LOG_TRACE("Resetting FOV and gain due to empty history and unsuccessful joint parsing.");
          image_provider_->ResetFOVAndGain();
          dont_allow_fov_change_until_new_dimensions_received = {image_provider_->GetVerticalFOVOffset(),
                                                                 image_provider_->GetVerticalFOVHeight()};
        }

        m_config_mutex.unlock();
      }

      if (metrics_.image_errors.contains(error)) {
        metrics_.image_errors.at(error)->Increment();
      } else {
        LOG_ERROR("missing error counter for: {}", JointModelErrorCodeToString(error));
      }

      /* only log the first image consecutive when the image processing fails */
      log_failed_image    = metrics_.image_consecutive_errors->Value() == 0;
      reason_failed_image = JointModelErrorCodeToSnakeCaseString(error);

      metrics_.image_consecutive_errors->Increment(1);
    }

    ImageLoggerEntry entry = {
        .image    = image.get(),
        .x_offset = static_cast<uint32_t>(image->GetHorizontalCropStart()),
        .y_offset = static_cast<uint32_t>(image->GetVerticalCropStart()),
    };

    if (log_failed_image) {
      image_logger_->LogImageError(entry, reason_failed_image);
    } else {
      image_logger_->LogImage(entry);
    }

    std::chrono::duration<double> const duration_seconds = std::chrono::steady_clock::now() - start_timstamp;
    metrics_.image_processing_time->Observe(duration_seconds.count());
  });
}

auto ScannerSliceToJointTrackingSlice(const core::scanner::JointSlice& slice)
    -> std::optional<core::joint_tracking::JointSlice> {
  if (slice.profile.points.size() != 7) {
    return std::nullopt;
  }

  std::array<core::joint_tracking::Coord, 7> points = {};

  uint32_t index = 0;
  for (const auto& point : slice.profile.points) {
    points[index++] = {point.x, point.y};
  }

  SliceConfidence confidence;
  switch (slice.num_walls_found) {
    case 2:
      confidence = SliceConfidence::HIGH;
      break;
    case 1:
      confidence = SliceConfidence::MEDIUM;
      break;
    default:
      confidence = SliceConfidence::LOW;
      break;
  };

  return core::joint_tracking::JointSlice(points, confidence);
}

auto CheckIfValueInRange(double value, double target, double range) -> bool {
  return value >= target - range && value <= target + range;
}

size_t ScannerImpl::CountOfReceivedImages() { return num_received; }

void ScannerImpl::Update() {
  m_buffer_mutex.lock();
  auto median_of_recent          = MedianOfRecentSlices();
  auto time_for_latest_in_buffer = joint_buffer_->GetLatestTimestamp().value_or(latest_sent);
  m_buffer_mutex.unlock();
  const auto update_available  = time_for_latest_in_buffer != latest_sent;
  const auto sufficient_slices = median_of_recent.has_value();

  if (update_available && sufficient_slices) {
    auto recent      = median_of_recent.value();
    auto maybe_slice = ScannerSliceToJointTrackingSlice(recent);

    if (maybe_slice.has_value()) {
      scanner_output_->ScannerOutput(maybe_slice.value(), std::array<Coord, 15>{}, recent.profile.area,
                                     recent.timestamp.time_since_epoch().count());
      latest_sent = time_for_latest_in_buffer;
    } else {
      LOG_WARNING("No slice sent due to missing ABW points.");
    }
  } else if (!sufficient_slices) {
    LOG_TRACE("Update requested but not enough recent slices were available.");
  } else if (!update_available) {
    LOG_TRACE("Update requested but no new slice information is available.");
  }
}

void ScannerImpl::UpdateJointGeometry(const JointProperties& properties) {
  m_buffer_mutex.lock();
  if (properties.upper_joint_width > 0. && properties.upper_joint_width_tolerance > 0. &&
      properties.left_joint_angle > 0. && properties.right_joint_angle > 0. && properties.groove_angle_tolerance > 0.) {
    empty_joint_properties_ = properties;
  } else {
    empty_joint_properties_ = std::nullopt;
    LOG_ERROR(
        "Faulty joint geometry: width {:.5f} width tolerance {:.5f} left angle {} right angle {:.5f} angle tolerance "
        "{:.5f}",
        properties.upper_joint_width, properties.upper_joint_width_tolerance, properties.left_joint_angle,
        properties.right_joint_angle, properties.groove_angle_tolerance);
  }
  m_buffer_mutex.unlock();
}

// Error code implementation
namespace {

struct ErrorCategory : std::error_category {
  auto name() const noexcept -> const char* final;          // NOLINT(*-use-nodiscard)
  auto message(int error_code) const -> std::string final;  // NOLINT(*-use-nodiscard)
  auto default_error_condition(int other) const noexcept    // NOLINT(*-use-nodiscard)
      -> std::error_condition final;                        // NOLINT(*-use-nodiscard)
};

auto ErrorCategory::name() const noexcept -> const char* { return "ScannerError"; }

auto ErrorCategory::message(int error_code) const -> std::string {
  switch (static_cast<ScannerErrorCode>(error_code)) {
    case ScannerErrorCode::NO_ERROR:
      return "No error";
    case ScannerErrorCode::NO_JOINT_PROPERTIES:
      return "Joint properties is not set";
  }
}

auto ErrorCategory::default_error_condition(int other) const noexcept -> std::error_condition {
  switch (static_cast<ScannerErrorCode>(other)) {
    default:
      return {other, *this};
  }
}

const ErrorCategory ERROR_CATEGORY{};

}  // namespace

[[maybe_unused]] auto core::scanner::make_error_code(ScannerErrorCode error_code)
    -> std::error_code {  // NOLINT(*-identifier-naming)
  return {static_cast<int>(error_code), ERROR_CATEGORY};
}
#ifndef DOCTEST_CONFIG_DISABLE
// NOLINTBEGIN(*-magic-number)

#include <doctest/doctest.h>

#include <boost/thread/thread.hpp>
#include <tuple>

#include "core/file/yaml.h"
#include "core/image/image_builder.h"
#include "core/scanner/image_logger_impl.h"
#include "core/scanner/joint_buffer/single_joint_buffer.h"

using core::file::Yaml;
using core::scanner::ScannerSensitivity;
using core::scanner::SingleJointBuffer;

namespace outcome = BOOST_OUTCOME_V2_NAMESPACE;

class ScannerOutputCBImpl : public core::scanner::ScannerOutputCB {
 public:
  void ScannerOutput(const core::joint_tracking::JointSlice& joint_slice,
                     const std::array<core::joint_tracking::Coord, 15>& line, std::optional<double> area,
                     uint64_t time_stamp) override {};
};

TEST_SUITE("Scanner") {
  class SimpleProvider : public core::scanner::ImageProvider {
   public:
    auto Start(enum ScannerSensitivity sensitivity) -> boost::outcome_v2::result<void> override {
      started_         = true;
      m_sending_thread = boost::thread(&SimpleProvider::SendImage, this);
      return outcome::success();
    }

    void Stop() override {
      m_sending_thread.join();
      started_ = false;
    }

    [[nodiscard]] auto Started() const -> bool override { return started_; }

    void ResetFOVAndGain() override {};
    void SetVerticalFOV(int offset_from_top, int height) override {};
    void AdjustGain(double factor) override {};
    auto GetVerticalFOVOffset() -> int override { return 0; };
    auto GetVerticalFOVHeight() -> int override { return 0; };
    auto GetSerialNumber() -> std::string override { return ""; };
    void SetOnImage(OnImage on_image) override { on_image_ = on_image; }

    void SendImage() {
      // Construct an image and immediately it to the image event handler
      using Eigen::Index;

      auto image_data = core::image::RawImageData(2500, 3500);

      // Return a straight line for now
      for (Index i = 0; i < image_data.cols(); i++) {
        image_data(1000, i) = static_cast<uint8_t>(255);
      }

      auto image = core::image::ImageBuilder::From(std::move(image_data), 0).Finalize().value();

      on_image_(std::move(image));
    }

   private:
    boost::thread m_sending_thread;
    bool started_;
    OnImage on_image_;
  };

  class CameraMock : public core::image::CameraModel {
    auto ImageToWorkspace(const core::image::PlaneCoordinates& coordinates, int vertical_crop_offset) const
        -> boost::outcome_v2::result<core::image::WorkspaceCoordinates> override {
      core::image::WorkspaceCoordinates wcs(3, coordinates.cols());
      wcs << coordinates.row(0).array(), coordinates.row(1).array(),
          Eigen::RowVectorXd::Zero(coordinates.cols()).array();

      return wcs;
    }

    auto WorkspaceToImage(const core::image::WorkspaceCoordinates& coordinates, int vertical_crop_offset) const
        -> boost::outcome_v2::result<core::image::PlaneCoordinates> override {
      core::image::PlaneCoordinates image(2, coordinates.cols());
      image << coordinates.row(0).array(), coordinates.row(1).array();

      return image;
    }
  };
  class JointModelMock : public core::scanner::JointModel {
   public:
    JointModelMock(const core::scanner::JointProperties& properties, core::image::CameraModelPtr camera_model)
        : JointModel(properties, std::move(camera_model)) {};

    auto Parse(core::image::Image& image, std::optional<core::scanner::JointProfile> median_profile,
               std::optional<core::scanner::JointProperties> empty_joint_properties)
        -> std::expected<std::tuple<core::scanner::JointProfile, core::image::WorkspaceCoordinates, uint64_t, uint64_t>,
                         core::scanner::JointModelErrorCode> {
      core::scanner::JointProfile profile;
      core::image::WorkspaceCoordinates coordinates;

      profile.points = {
          core::scanner::Point{0.1, 0.1  },
          core::scanner::Point{0.3, -0.1 },
          core::scanner::Point{0.4, -0.12},
          core::scanner::Point{0.5, -0.14},
          core::scanner::Point{0.6, -0.12},
          core::scanner::Point{0.7, -0.1 },
          core::scanner::Point{0.8, 0.1  }
      };

      return std::make_tuple(profile, coordinates, 0, 0);
    }
  };

  class ScannerExposed : public core::scanner::ScannerImpl {
   public:
    ScannerExposed(core::scanner::ImageProvider* image_provider, core::scanner::JointModelPtr joint_model,
                   core::scanner::JointBufferPtr joint_buffer, const core::scanner::LaserCallback& laser_toggle,
                   ScannerOutputCBImpl* scanner_ouput_cb, core::scanner::ImageLogger* image_logger,
                   prometheus::Registry* registry)
        : ScannerImpl(image_provider, std::move(joint_buffer), laser_toggle, scanner_ouput_cb, std::move(joint_model),
                      image_logger, registry) {}

    using core::scanner::ScannerImpl::Update;
  };

  TEST_CASE("Field-of-View changes") {
    auto [o1, h1] = ScannerImpl::NewOffsetAndHeight(100, 400);
    CHECK_EQ(o1, 0);
    CHECK_EQ(h1, MINIMUM_FOV_HEIGHT);

    auto [o2, h2] = ScannerImpl::NewOffsetAndHeight(100, 500);
    CHECK_EQ(o2, 100 - WINDOW_MARGIN);
    CHECK_EQ(h2, 500 + WINDOW_MARGIN);

    auto [o3, h3] = ScannerImpl::NewOffsetAndHeight(0, 300);
    CHECK_EQ(o3, 0);
    CHECK_EQ(h3, MINIMUM_FOV_HEIGHT);
  }

  TEST_CASE("Scan single image") {
    using namespace std::chrono_literals;

    std::unique_ptr<core::scanner::ImageProvider> provider =
        std::unique_ptr<core::scanner::ImageProvider>(new SimpleProvider());
    auto* provider_raw = dynamic_cast<SimpleProvider*>(provider.get());

    auto camera = std::unique_ptr<core::image::CameraModel>(new CameraMock());

    auto joint_buffer = std::make_unique<SingleJointBuffer>();
    auto properties   = core::scanner::JointProperties();

    auto joint_model = core::scanner::JointModelPtr(new JointModelMock(properties, std::move(camera)));

    ScannerOutputCBImpl scanner_output_cb;

    auto registry = std::make_shared<prometheus::Registry>();

    auto image_logger = core::scanner::ImageLoggerImpl(0);
    auto scanner      = ScannerExposed(
        provider_raw, std::move(joint_model), std::move(joint_buffer), [](bool state) {}, &scanner_output_cb,
        &image_logger, registry.get());

    auto on_image = [&scanner](std::unique_ptr<core::image::Image> img) -> void {
      scanner.ImageGrabbed(std::move(img));
    };
    provider->SetOnImage(on_image);

    scanner.Update();

    CHECK_EQ(scanner.GetJointBuffer()->GetNumberOfSlices(), 0);

    auto result = provider_raw->Start(core::scanner::ScannerSensitivity::NORMAL);

    CHECK(result);

    provider_raw->Stop();

    boost::this_thread::sleep(boost::posix_time::milliseconds(200));

    scanner.Update();

    CHECK_EQ(scanner.GetJointBuffer()->GetNumberOfSlices(), 1);
  }
}

// NOLINTEND(*-magic-number)
#endif
