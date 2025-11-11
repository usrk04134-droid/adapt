#include "scanner/image_provider/camera_simulation.h"

#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <set>
#include <string>
#include <system_error>
#include <tuple>
#include <utility>
#include <vector>

#include "common/logging/application_log.h"
#include "scanner/core/scanner_types.h"
#include "scanner/image/image.h"
#include "scanner/image/image_builder.h"
#include "scanner/image/tiff_handler_impl.h"
#include "scanner/image_provider/image_provider_configuration.h"

namespace {
auto const IMAGE_SIMULATION_DELAY_MS = 15;
}  // namespace

namespace outcome = BOOST_OUTCOME_V2_NAMESPACE;
namespace fs      = std::filesystem;

namespace scanner::image_provider {

using namespace std::chrono_literals;
using std::chrono::duration_cast;
using std::chrono::milliseconds;

CameraSimulation::CameraSimulation(const SimConfig& config, bool loop)
    : started_(false), real_time_mode_(config.realtime), loop_(loop), previous_time_stamp_(0), offset_(0), height_(0) {
  auto search_path                          = fs::path(config.images_path);
  std::vector<std::string> supported_images = {"bmp", "tiff"};

  if (fs::is_regular_file(search_path)) {
    for (auto& supported_image : supported_images) {
      if (search_path.has_extension() &&
          (search_path.extension().string() == std::string(".").append(supported_image))) {
        image_files_.push_back(search_path);
      }
    }
    if (image_files_.empty()) {
      LOG_ERROR("Unrecognized file format");
    }
  } else if (fs::is_directory(search_path)) {
    std::map<std::chrono::milliseconds, std::set<fs::directory_entry>> sort_by_time;

    for (const auto& entry : fs::directory_iterator(search_path)) {
      if (fs::is_directory(entry)) {
        continue;
      }

      auto time = duration_cast<milliseconds>(entry.last_write_time().time_since_epoch());
      sort_by_time[time].insert(entry);
    }

    for (auto const& [time, entries] : sort_by_time) {
      for (const auto& entry : entries) {
        const auto& path = entry.path();
        for (auto& supported_image : supported_images) {
          if (path.has_extension() && (path.extension().string() == std::string(".").append(supported_image))) {
            LOG_INFO("Added image file {}", path.string());
            image_files_.push_back(entry);
          }
        }
      }
    }
  } else {
    LOG_ERROR("Unrecognized file format, path: {}", search_path.string());
  }
}

CameraSimulation::~CameraSimulation() = default;

auto CameraSimulation::Start(enum scanner::ScannerSensitivity /*sensitivity*/) -> boost::outcome_v2::result<void> {
  if (on_image_ == NULL) {
    LOG_ERROR("Starting the camera simulation but the image handler has not been set.");
    return outcome::failure(CameraSimulationErrorCode::IMAGE_EVENT_HANDLER_NOT_SET);
  }
  LOG_TRACE("Starting image grabbing.");
  if (image_files_.empty()) {
    LOG_ERROR("Starting simulated image grabbing without images");
    return CameraSimulationErrorCode::NO_IMAGES_FOUND;
  }

  // Just a reference to self - make sure to call Stop before this goes out of scope
  grabbing_thread_ = boost::thread(&CameraSimulation::Run, this);
  started_         = true;
  return outcome::success();
}

void CameraSimulation::Stop() {
  LOG_TRACE("Stopping image grabbing.");
  if (!started_) {
    LOG_WARNING("Attempting to stop image grabbing, but grabbing is not running.");
    return;
  }
  grabbing_thread_.interrupt();
  LOG_TRACE("Grabbing thread interrupted, joining…");
  grabbing_thread_.join();
  LOG_TRACE("Grabbing thread joined.");
  started_ = false;
}

void CameraSimulation::Run() {
  // Get an image
  while (current_image_ < image_files_.size()) {
    // Measure process time for fetching file from disk
    auto start      = std::chrono::high_resolution_clock::now();
    auto image_data = GetImage();
    auto process_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start)
            .count();
    ;

    auto maybe_image      = std::move(get<0>(image_data));
    auto maybe_time_stamp = std::get<1>(image_data);

    if (!maybe_image.has_value()) {
      LOG_ERROR("Retrieve simulated image failed.");
      continue;
    }

    assert(IMAGE_SIMULATION_DELAY_MS > 0);
    uint32_t delay = IMAGE_SIMULATION_DELAY_MS;

    if (real_time_mode_ && maybe_time_stamp.has_value()) {
      loop_           = false;
      auto time_stamp = maybe_time_stamp.value();

      if (previous_time_stamp_ == 0) {
        // First image
        previous_time_stamp_ = time_stamp;
      } else {
        // Compensate for process time of reading from file
        if ((time_stamp - previous_time_stamp_) > process_time) {
          delay = time_stamp - previous_time_stamp_ - process_time;
          LOG_TRACE("Delay image {} ms", delay);
        } else {
          LOG_WARNING("Process time for fetching image took too long.");
          // Process the image right away
          delay = 0;
        }

        previous_time_stamp_ = time_stamp;
      }
    } else if (real_time_mode_) {
      LOG_WARNING("Try to run real time simulation but no timestamp in images.");
    }

    // This interval must not be 0. If it is zero then the thread will never stop to listen
    // for signals.
    boost::this_thread::sleep(boost::posix_time::milliseconds(delay));

    auto image = std::move(maybe_image.value());

    // Set time stamp, it takes ~90ms until the image is received
    image->SetTimestamp(std::chrono::high_resolution_clock::now() - std::chrono::milliseconds(90));

    // Send the image to the image event handler
    // m_handler->ImageGrabbed(std::move(image));
    on_image_(std::move(image));
  }
}

void CameraSimulation::ResetFOVAndGain() { SetVerticalFOV(0, 0); }

void CameraSimulation::SetVerticalFOV(int offset_from_top, int height) {
  offset_ = offset_from_top;
  height_ = height;
}

void CameraSimulation::AdjustGain(double factor) {}

auto CameraSimulation::GetVerticalFOVOffset() -> int { return offset_; };

auto CameraSimulation::GetVerticalFOVHeight() -> int { return height_; };

auto CameraSimulation::GetImage()
    -> std::tuple<std::optional<std::unique_ptr<scanner::image::Image>>, std::optional<uint32_t>> {
  using cv::COLOR_RGB2GRAY;
  using cv::imread;
  using cv::Mat;

  if (!Started()) {
    return {std::nullopt, std::nullopt};
  }

  auto image_vector       = std::vector(image_files_.begin(), image_files_.end());
  const auto& image_file  = image_vector[current_image_];
  current_image_         += 1;

  LOG_TRACE("Grabbing image {}.", image_file.string());

  if (current_image_ >= image_files_.size() && loop_) {
    LOG_TRACE("Restarting image loop");
    current_image_ = 0;
  }

  auto time_stamp      = scanner::image::ReadTimestamp(image_file);
  auto [fov_x, fov_y]  = scanner::image::ReadFovOffset(image_file).value_or(std::make_tuple(0, 0));
  auto grayscale_image = imread(image_file, cv::IMREAD_GRAYSCALE);

  if (grayscale_image.empty()) {
    return {std::nullopt, time_stamp};
  }

  auto maybe_image = image::ImageBuilder::From(grayscale_image, image_file.filename(), fov_y).Finalize();

  if (!maybe_image.has_value()) {
    LOG_ERROR("Error while building image: {}", maybe_image.error().to_string());
    return {std::nullopt, std::nullopt};
  }

  return {std::move(maybe_image.value()), time_stamp};
}

auto CameraSimulation::Started() const -> bool { return started_; }

auto CameraSimulation::GetSerialNumber() -> std::string { return ""; }

}  // namespace scanner::image_provider

// Error code implementation
namespace {

struct ErrorCategory : std::error_category {
  auto name() const noexcept -> const char* final;          // NOLINT(*-use-nodiscard)
  auto message(int error_code) const -> std::string final;  // NOLINT(*-use-nodiscard)
  auto default_error_condition(int other) const noexcept    // NOLINT(*-use-nodiscard)
      -> std::error_condition final;                        // NOLINT(*-use-nodiscard)
};

auto ErrorCategory::name() const noexcept -> const char* { return "CameraSimulationErrorCode"; }

auto ErrorCategory::message(int error_code) const -> std::string {
  switch (static_cast<scanner::image_provider::CameraSimulationErrorCode>(error_code)) {
    case scanner::image_provider::CameraSimulationErrorCode::NO_ERROR:
      return "No error";
    case scanner::image_provider::CameraSimulationErrorCode::NO_IMAGES_FOUND:
      return "No images found";
    case scanner::image_provider::CameraSimulationErrorCode::IMAGE_EVENT_HANDLER_NOT_SET:
      return "Image event handler not set";
  }
}

auto ErrorCategory::default_error_condition(int other) const noexcept -> std::error_condition {
  switch (static_cast<scanner::image_provider::CameraSimulationErrorCode>(other)) {
    default:
      return {other, *this};
  }
}

const ErrorCategory ERROR_CATEGORY{};

}  // namespace

[[maybe_unused]] auto scanner::image_provider::make_error_code(CameraSimulationErrorCode error_code)
    -> std::error_code {  // NOLINT(*-identifier-naming)
  return {static_cast<int>(error_code), ERROR_CATEGORY};
}
