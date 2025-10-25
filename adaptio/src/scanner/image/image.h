#pragma once

#include <boost/outcome.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <chrono>
#include <cstdint>
#include <eigen3/Eigen/Eigen>
#include <memory>

#include "scanner/image/image_types.h"

namespace scanner::image {

class ImageBuilder;
class Image;

using ImagePtr = std::unique_ptr<Image>;

template <typename PixelType>
using NewValue = std::function<PixelType(PixelType pixel_value, Eigen::Index row, Eigen::Index column)>;

class Image {
  friend class scanner::image::ImageBuilder;

 public:
  using Timestamp = std::chrono::time_point<std::chrono::high_resolution_clock>;

  Image(const Image &)                         = default;
  auto operator=(const Image &) -> Image &     = default;
  Image(Image &&) noexcept                     = default;
  auto operator=(Image &&) noexcept -> Image & = default;

  ~Image();

  auto Data() const -> const RawImageData &;

  auto AsBytes() -> std::vector<uint8_t>;

  auto GetUuid() -> boost::uuids::uuid;

  auto GetTimestamp() const -> Timestamp;
  void SetTimestamp(Timestamp t);

  auto GetImageName() const -> std::string;

  auto SetHorizontalCrop(int start_column, int stop_column) -> void {
    start_col_ = start_column;
    stop_col_  = stop_column;

    if (start_column < 0 || start_column >= data_.cols()) {
      start_col_ = 0;
    }
    if (stop_column >= data_.cols()) {
      stop_col_ = data_.cols();
    }
  };

  auto Cols() const -> int {
    if (start_col_ != 0) {
      return stop_col_ - start_col_;
    }

    return data_.cols();
  };

  auto StartCol() const -> int { return start_col_; };
  auto StopCol() const -> int {
    if (start_col_ != 0) {
      return stop_col_;
    }

    return data_.cols();
  };

  // Returns the starting column index of the horizontal crop
  auto GetHorizontalCropStart() const -> int { return start_col_; };

  /**
   * Applies the given lambda to each pixel of this image (in place)
   * @tparam PixelType The type of the matrix elements
   * @param new_value A lambda that describes the new value
   */
  template <typename PixelType>
  void PixelFilter(const NewValue<PixelType> &new_value) {
    using Eigen::Index;
    for (Index row = 0; row < data_.rows(); row++) {
      for (Index column = 0; column < data_.cols(); column++) {
        data_(row, column) = new_value(data_(row, column), row, column);
      }
    }
  }

  auto ClearPixelBelowThreshold(uint8_t threshold) const -> RawImageData {
    return data_.unaryExpr([threshold](uint8_t pixel_val) { return (pixel_val < threshold) ? 0 : pixel_val; })
        .cast<uint8_t>();
  }
  auto GetVerticalCropStart() const -> int { return vertical_crop_start_; };

 private:
  RawImageData data_;
  boost::uuids::uuid uuid_;
  Timestamp timestamp_;
  std::string img_name_;

  int vertical_crop_start_ = 0;

  explicit Image(RawImageData matrix);
  Image(RawImageData matrix, const std::string &img_name);
  Image(RawImageData matrix, Timestamp timestamp);
  int start_col_ = 0;
  int stop_col_  = 0;
};

using ImagePtr = std::unique_ptr<Image>;

}  // namespace scanner::image
