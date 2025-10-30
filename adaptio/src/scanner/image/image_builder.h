#pragma once

#include <filesystem>
#include <opencv2/opencv.hpp>

#include "scanner/image/image.h"

namespace scanner::image {

enum class ImageBuilderErrorCode : uint32_t {
  NO_ERROR = 0,
  FILE_NOT_FOUND,
  INTERNAL_ERROR,
};

// NOLINTNEXTLINE(*-identifier-naming)
[[maybe_unused]] auto make_error_code(ImageBuilderErrorCode) -> std::error_code;

class ImageBuilder {
 public:
  static auto From(std::filesystem::path path) -> ImageBuilder;
  static auto From(const cv::Mat& matrix, int vertical_crop_start, int horizontal_crop_start) -> ImageBuilder;
  static auto From(const cv::Mat& matrix, const std::string& img_name, int vertical_crop_start,
                   int horizontal_crop_start) -> ImageBuilder;
  static auto From(RawImageData image, int vertical_crop_start, int horizontal_crop_start) -> ImageBuilder;

  auto Finalize() -> boost::outcome_v2::result<std::unique_ptr<Image>>;

 private:
  std::optional<ImageBuilderErrorCode> error_code_ = std::nullopt;
  std::optional<std::unique_ptr<Image>> image_     = std::nullopt;
};

};  // namespace scanner::image

namespace std {
template <>
struct is_error_code_enum<scanner::image::ImageBuilderErrorCode> : true_type {};
}  // namespace std
