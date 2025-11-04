#include "scanner/scanner_impl.h"

#include <prometheus/counter.h>
#include <prometheus/gauge.h>
#include <prometheus/histogram.h>
#include <prometheus/registry.h>

#include <array>
#include <boost/asio/post.hpp>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <Eigen/Core>
#include <expected>
#include <memory>
#include <optional>
#include <string>
#include <system_error>
#include <tuple>
#include <utility>
#include <vector>

#include "common/logging/application_log.h"
#include "scanner/image/camera_model.h"
#include "scanner/image/image.h"
#include "scanner/image/image_types.h"  // IWYU pragma: keep
#include "scanner/image_logger/image_logger.h"
#include "scanner/image_provider/image_provider.h"
#include "scanner/joint_buffer/joint_buffer.h"
#include "scanner/joint_model/joint_model.h"
#include "scanner/scanner.h"
#include "scanner/scanner_types.h"
#include "scanner/slice_provider/slice_provider.h"

namespace scanner {
// #define DEBUG_RESULT_OUTPUT 1

#if defined(DEBUG_RESULT_OUTPUT)
#include <format>
#include <fstream>
#include <ios>
#include <ostream>
#endif

ScannerImpl::ScannerImpl(image_provider::ImageProvider* image_provider, slice_provider::SliceProviderPtr slice_provider,
                         LaserCallback laser_toggle, ScannerOutputCB* scanner_output,
                         joint_model::JointModelPtr joint_model, image_logger::ImageLogger* image_logger,
                         prometheus::Registry* registry)
    : image_provider_(image_provider),
      joint_model_(std::move(joint_model)),
      slice_provider_(std::move(slice_provider)),
      laser_toggle_(std::move(laser_toggle)),
      scanner_output_(scanner_output),
      image_logger_(image_logger),
      m_threadpool(12),
      store_image_data_(false) {
  SetupMetrics(registry);

  post_ = [this](std::function<void()> fn) { boost::asio::post(m_threadpool, std::move(fn)); };
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
        joint_model::JointModelErrorCode::NO_ERROR,
        joint_model::JointModelErrorCode::SURFACE_NOT_FOUND,
        joint_model::JointModelErrorCode::WEDGE_FIT_FAILED,
        joint_model::JointModelErrorCode::GROOVE_BOTTOM_NOT_FOUND,
        joint_model::JointModelErrorCode::GROOVE_WALL_CENTROIDS_NOT_FOUND,
        joint_model::JointModelErrorCode::MISSING_WEDGE_HISTORY,
        joint_model::JointModelErrorCode::INVALID_SNAKE,
        joint_model::JointModelErrorCode::INVALID_WALL_HEIGHT_DIFFERENCE,
        joint_model::JointModelErrorCode::SURFACE_ANGLE_TOLERANCE_EXCEEDED,
        joint_model::JointModelErrorCode::JOINT_WIDTH_OUT_OF_TOLERANCE,
        joint_model::JointModelErrorCode::TWO_WALLS_NOT_FOUND,
    };

    for (auto code : codes) {
      metrics_.image_errors.emplace(code, &counter.Add({
                                              {"error", joint_model::JointModelErrorCodeToSnakeCaseString(code)}
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
  auto on_image = [this](std::unique_ptr<image::Image> img) -> void { ImageGrabbed(std::move(img)); };
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
  updated_properties_         = {};
  maybe_abw0_abw6_horizontal_ = {};
  image_provider_->SetOnImage(nullptr);
  dont_allow_fov_change_until_new_dimensions_received = std::nullopt;
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

void ScannerImpl::ImageGrabbed(std::unique_ptr<image::Image> image) {
  auto sp_image = std::shared_ptr<image::Image>(std::move(image));
  post_([this, image = std::move(sp_image)]() {
    try {
      // When image capture is faster than parsing we need to be able to evaluate concurrently across multiple cores
      // This means that Parse should be a constant function. Any state should be recoverable from the latest slice.

      auto const start_timstamp = std::chrono::steady_clock::now();
      auto log_failed_image     = false;
      std::string reason_failed_image;

      m_buffer_mutex.lock();
      auto median_profile             = slice_provider_->GetSlice();
      auto updated_properties         = updated_properties_;
      auto maybe_abw0_abw6_horizontal = maybe_abw0_abw6_horizontal_;
      auto use_approximation          = slice_provider_->SliceDegraded();
      m_buffer_mutex.unlock();

      auto result = joint_model_->Parse(*image.get(), median_profile, updated_properties, use_approximation,
                                        maybe_abw0_abw6_horizontal);

      num_received++;

      if (result) {
        auto [profile, centroids_wcs, processing_time, num_walls_found] = *result;
        LOG_TRACE("Processed image {} in {} ms.", image->GetImageName(), processing_time);
        joint_buffer::JointSlice slice = {.uuid                = image->GetUuid(),
                                          .timestamp           = image->GetTimestamp(),
                                          .image_name          = image->GetImageName(),
                                          .profile             = profile,
                                          .num_walls_found     = num_walls_found,
                                          .processing_time     = processing_time,
                                          .vertical_crop_start = image->GetVerticalCropStart(),
                                          .approximation_used  = profile.approximation_used};
        if (store_image_data_) {
          // Store image data only if necessary. Not needed when running Adaptio
          slice.image_data = image->Data();
          slice.centroids  = centroids_wcs;
        }

        m_buffer_mutex.lock();
        slice_provider_->AddSlice(slice);
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
          LOG_ERROR("missing error counter for: {}", joint_model::JointModelErrorCodeToString(error));
        }

        /* only log the first image consecutive when the image processing fails */
        log_failed_image    = metrics_.image_consecutive_errors->Value() == 0;
        reason_failed_image = joint_model::JointModelErrorCodeToSnakeCaseString(error);

        metrics_.image_consecutive_errors->Increment(1);
      }

      image_logger::ImageLoggerEntry entry = {
          .image    = image.get(),
          .x_offset = 0,
          .y_offset = static_cast<uint32_t>(image->GetVerticalCropStart()),
      };

      if (log_failed_image) {
        image_logger_->LogImageError(entry, reason_failed_image);
      } else {
        image_logger_->LogImage(entry);
      }

      std::chrono::duration<double> const duration_seconds = std::chrono::steady_clock::now() - start_timstamp;
      metrics_.image_processing_time->Observe(duration_seconds.count());
    } catch (const std::exception& e) {
      LOG_ERROR("Exception in image processing thread for image {}: {}", 
                image ? image->GetImageName() : "unknown", e.what());
      // Increment error counter to track consecutive errors
      if (metrics_.image_consecutive_errors) {
        metrics_.image_consecutive_errors->Increment(1);
      }
      // Log error for the specific error type if we have an image
      if (image && metrics_.image_errors.contains(joint_model::JointModelErrorCode::INVALID_SNAKE)) {
        metrics_.image_errors.at(joint_model::JointModelErrorCode::INVALID_SNAKE)->Increment();
      }
    } catch (...) {
      LOG_ERROR("Unknown exception in image processing thread for image {}", 
                image ? image->GetImageName() : "unknown");
      // Increment error counter to track consecutive errors
      if (metrics_.image_consecutive_errors) {
        metrics_.image_consecutive_errors->Increment(1);
      }
    }
  });
}

auto CheckIfValueInRange(double value, double target, double range) -> bool {
  return value >= target - range && value <= target + range;
}

auto ScannerImpl::CountOfReceivedImages() -> size_t { return num_received; }

void ScannerImpl::Update() {
  m_buffer_mutex.lock();
  auto tracking_data = slice_provider_->GetTrackingSlice();
  m_buffer_mutex.unlock();

  if (tracking_data.has_value()) {
    auto [groove, confidence, time_stamp] = tracking_data.value();

    scanner_output_->ScannerOutput(groove, time_stamp, confidence);
  } else {
    // This should not happen
    LOG_ERROR("No slice sent due to missing ABW points.");
  }
}

void ScannerImpl::UpdateJointApproximation(const joint_model::JointProperties& properties,
                                           const std::tuple<double, double>& abw0_abw6_horizontal) {
  m_buffer_mutex.lock();
  auto [abw0_horizontal, abw6_horizontal] = abw0_abw6_horizontal;
  if (abw6_horizontal > abw0_horizontal) {
    maybe_abw0_abw6_horizontal_ = abw0_abw6_horizontal;
  }

  if (properties.upper_joint_width > 0. && properties.upper_joint_width_tolerance > 0. &&
      properties.left_joint_angle > 0. && properties.right_joint_angle > 0. && properties.groove_angle_tolerance > 0.) {
    updated_properties_ = properties;
  } else {
    updated_properties_ = std::nullopt;
    LOG_ERROR(
        "Faulty joint geometry: width {:.5f} width tolerance {:.5f} left angle {} right angle {:.5f} angle tolerance "
        "{:.5f}",
        properties.upper_joint_width, properties.upper_joint_width_tolerance, properties.left_joint_angle,
        properties.right_joint_angle, properties.groove_angle_tolerance);
  }
  m_buffer_mutex.unlock();

  if (updated_properties_) {
    const auto properties = updated_properties_.value();
    LOG_TRACE("Joint properties updated: width {:.5f} left angle {:.5f} right angle {:.5f}",
              properties.upper_joint_width, properties.left_joint_angle, properties.right_joint_angle);
  }

  if (maybe_abw0_abw6_horizontal_) {
    auto [abw0x, abw6x] = maybe_abw0_abw6_horizontal_.value();
    LOG_TRACE("Joint approximation updated: abw0 horizontal {:.5f} abw6 horizontal {:.5f}", abw0x, abw6x);
  }
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

[[maybe_unused]] auto make_error_code(ScannerErrorCode error_code) -> std::error_code {  // NOLINT(*-identifier-naming)
  return {static_cast<int>(error_code), ERROR_CATEGORY};
}

void ScannerImpl::SetPostExecutorForTests(std::function<void(std::function<void()>)> exec) { post_ = std::move(exec); }
}  // namespace scanner
