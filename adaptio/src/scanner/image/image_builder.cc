#include "scanner/image/image_builder.h"

#include <opencv2/core/hal/interface.h>

#include <boost/outcome/result.hpp>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <Eigen/Core>
#include <filesystem>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <string>
#include <system_error>
#include <tuple>
#include <utility>

#include "scanner/image/image.h"
#include "scanner/image/image_types.h"
#include "scanner/image/tiff_handler_impl.h"

namespace scanner::image {

auto ImageBuilder::From(std::filesystem::path path) -> ImageBuilder {
  ImageBuilder builder;

  if (!exists(path) || !is_regular_file(path)) {
    builder.error_code_ = ImageBuilderErrorCode::FILE_NOT_FOUND;
  }

  auto [fov_x, fov_y] = ReadFovOffset(path.string()).value_or(std::make_tuple(0, 0));

  auto opencv_image = cv::imread(path.string(), cv::IMREAD_GRAYSCALE);

  RawImageData raw_image;
  auto rows    = static_cast<uint64_t>(opencv_image.rows);
  auto columns = static_cast<uint64_t>(opencv_image.cols);

  auto* data = (uchar*)malloc(rows * columns * sizeof(uchar));  // NOLINT(*-owning-memory)
  memcpy(data, opencv_image.data, rows * columns);

  new (&raw_image) Eigen::Map<RawImageData>(data, rows, columns);

  ImagePtr image_ptr;
  image_ptr.reset(new Image(std::move(raw_image)));
  image_ptr->vertical_crop_start_   = static_cast<int>(fov_y);
  image_ptr->horizontal_crop_start_ = static_cast<int>(fov_x);

  builder.image_ = std::move(image_ptr);

  return builder;
}

auto ImageBuilder::From(const cv::Mat& matrix, int vertical_crop_start, int horizontal_crop_start) -> ImageBuilder {
  ImageBuilder builder;

  auto image_data = RawImageData(Eigen::Map<RawImageData>(matrix.data, matrix.rows, matrix.cols));

  ImagePtr image_ptr;
  image_ptr.reset(new Image(std::move(image_data)));
  image_ptr->vertical_crop_start_   = vertical_crop_start;
  image_ptr->horizontal_crop_start_ = horizontal_crop_start;

  builder.image_ = std::move(image_ptr);

  return builder;
}

auto ImageBuilder::From(const cv::Mat& matrix, const std::string& img_name, int vertical_crop_start,
                        int horizontal_crop_start) -> ImageBuilder {
  ImageBuilder builder;

  auto image_data = RawImageData(Eigen::Map<RawImageData>(matrix.data, matrix.rows, matrix.cols));

  ImagePtr image_ptr;
  image_ptr.reset(new Image(std::move(image_data), img_name));
  image_ptr->vertical_crop_start_   = vertical_crop_start;
  image_ptr->horizontal_crop_start_ = horizontal_crop_start;

  builder.image_ = std::move(image_ptr);

  return builder;
}

auto ImageBuilder::From(RawImageData image, int vertical_crop_start, int horizontal_crop_start) -> ImageBuilder {
  ImageBuilder builder;

  ImagePtr image_ptr;
  image_ptr.reset(new Image(std::move(image)));
  image_ptr->vertical_crop_start_   = vertical_crop_start;
  image_ptr->horizontal_crop_start_ = horizontal_crop_start;

  builder.image_ = std::move(image_ptr);

  return builder;
}

auto ImageBuilder::Finalize() -> boost::outcome_v2::result<std::unique_ptr<Image>> {
  if (error_code_ != std::nullopt) {
    return error_code_.value();
  }

  if (image_ != std::nullopt) {
    return std::move(image_.value());
  }

  return ImageBuilderErrorCode::INTERNAL_ERROR;
}
}  // namespace scanner::image

// Error code implementation
namespace {

struct ErrorCategory : std::error_category {
  auto name() const noexcept -> const char* final;          // NOLINT(*-use-nodiscard)
  auto message(int error_code) const -> std::string final;  // NOLINT(*-use-nodiscard)
  auto default_error_condition(int other) const noexcept    // NOLINT(*-use-nodiscard)
      -> std::error_condition final;                        // NOLINT(*-use-nodiscard)
};

auto ErrorCategory::name() const noexcept -> const char* { return "CameraModelError"; }

auto ErrorCategory::message(int error_code) const -> std::string {
  switch (static_cast<scanner::image::ImageBuilderErrorCode>(error_code)) {
    case scanner::image::ImageBuilderErrorCode::NO_ERROR:
      return "No error";
    case scanner::image::ImageBuilderErrorCode::FILE_NOT_FOUND:
      return "File not found";
    case scanner::image::ImageBuilderErrorCode::INTERNAL_ERROR:
      return "Internal error";
  }
}

auto ErrorCategory::default_error_condition(int other) const noexcept -> std::error_condition {
  switch (static_cast<scanner::image::ImageBuilderErrorCode>(other)) {
    default:
      return {other, *this};
  }
}

const ErrorCategory ERROR_CATEGORY{};

}  // namespace

[[maybe_unused]] auto scanner::image::make_error_code(scanner::image::ImageBuilderErrorCode error_code)
    -> std::error_code {  // NOLINT(*-identifier-naming)
  return {static_cast<int>(error_code), ERROR_CATEGORY};
}
