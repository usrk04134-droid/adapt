#include "scanner/image/image.h"

#include <boost/uuid/uuid.hpp>
#include <chrono>
#include <cstdint>
#include <Eigen/Core>
#include <string>
#include <utility>
#include <vector>

#include "scanner/image/image_types.h"

using Eigen::Index;
using Eigen::last;
using Eigen::Matrix3d;
using Eigen::RowVectorXd;

namespace scanner::image {
Image::Image(RawImageData matrix)
    : data_(std::move(matrix)),
      uuid_(boost::uuids::random_generator()()),
      timestamp_(std::chrono::high_resolution_clock::now()) {}

Image::Image(RawImageData matrix, const std::string& img_name)
    : data_(std::move(matrix)),
      uuid_(boost::uuids::random_generator()()),
      timestamp_(std::chrono::high_resolution_clock::now()),
      img_name_(img_name) {}

Image::Image(RawImageData matrix, Timestamp timestamp)
    : data_(std::move(matrix)), uuid_(boost::uuids::random_generator()()), timestamp_(timestamp) {}

Image::~Image() = default;

auto Image::Data() const -> const RawImageData& { return data_; }

auto Image::AsBytes() -> std::vector<uint8_t> { return {}; }

auto Image::GetUuid() -> boost::uuids::uuid { return uuid_; }

auto Image::GetTimestamp() const -> Timestamp { return timestamp_; }
void Image::SetTimestamp(Timestamp t) { timestamp_ = t; }

auto Image::GetImageName() const -> std::string { return img_name_; }
}  // namespace scanner::image
