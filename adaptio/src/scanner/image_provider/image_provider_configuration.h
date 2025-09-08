#pragma once

#include <cstdint>
#include <string>

namespace scanner::image_provider {

enum class ImageProviderType : uint32_t {
  BASLER = 0,
  SIMULATION,
  UNDEFINED,
};

struct SimConfig {
  bool realtime;
  std::string images_path;
};

struct Fov {
  int64_t width;
  int64_t offset_x;
  int64_t height;
  int64_t offset_y;
};

struct BaslerConfig {
  double gain;           // dB
  double exposure_time;  // us
};

struct ImageProviderConfigData {
  ImageProviderType image_provider = ImageProviderType::BASLER;
  struct Fov fov;
  BaslerConfig basler_config = {};
  SimConfig sim_config       = {};
};

}  // namespace scanner::image_provider
