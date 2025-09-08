#pragma once

#include <cstdint>
#include <eigen3/Eigen/Eigen>

namespace scanner::image {

using RawImageData     = Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using GrayScaleData    = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using RawImageDataRef  = Eigen::Ref<RawImageData>;
using GrayScaleDataRef = Eigen::Ref<GrayScaleData>;

struct ImageProcessing {
  size_t cog_search_width;
};

}  // namespace scanner::image
