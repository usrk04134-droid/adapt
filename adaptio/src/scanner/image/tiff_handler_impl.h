#pragma once

#include <tiff.h>

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

#include "scanner/image/image.h"
#include "scanner/image/tiff_handler.h"
#include "tiffio.h"

namespace scanner::image {

enum {
  TIFFTAG_X_OFFSET = 65000,
  TIFFTAG_Y_OFFSET,
  TIFFTAG_TIMESTAMP_RFC3339,
  TIFFTAG_TIMESTAMP,
  TIFFTAG_SCANNER_CALIBRATION,
  TIFFTAG_JOINT_GEOMETRY
};

auto inline ReadTimestamp(const std::filesystem::path& file_path) -> std::optional<uint32_t> {
  if (!file_path.extension().string().contains("tiff")) {
    return std::nullopt;
  }
  TIFF* tif = TIFFOpen(file_path.c_str(), "r");
  if (tif == nullptr) {
    return std::nullopt;
  }

  // There should only be one image in the directory
  while (TIFFReadDirectory(tif) != 0) {
  };

  char time_stamp_str[30] = {0};
  char* ts_p              = time_stamp_str;

  if (TIFFGetField(tif, TIFFTAG_TIMESTAMP, &ts_p)) {
    uint32_t time_stamp = std::stol(reinterpret_cast<char*>(ts_p));
    TIFFClose(tif);

    return time_stamp;
  }
  TIFFClose(tif);

  return std::nullopt;
};

auto inline ReadFovOffset(const std::filesystem::path& file_path) -> std::optional<std::tuple<uint32_t, uint32_t>> {
  if (!file_path.extension().string().contains("tiff")) {
    return std::nullopt;
  }
  TIFF* tif = TIFFOpen(file_path.c_str(), "r");
  if (tif == nullptr) {
    return std::nullopt;
  }

  // There should only be one image in the directory
  while (TIFFReadDirectory(tif) != 0) {
  };

  uint16_t x_offset_raw = 0;
  uint16_t y_offset_raw = 0;

  if (!TIFFGetField(tif, TIFFTAG_X_OFFSET, &x_offset_raw)) {
    TIFFClose(tif);
    return {};
  }

  if (!TIFFGetField(tif, TIFFTAG_Y_OFFSET, &y_offset_raw)) {
    TIFFClose(tif);
    return {};
  }

  TIFFClose(tif);

  return {
      {static_cast<uint32_t>(x_offset_raw), static_cast<uint32_t>(y_offset_raw)}
  };
};

class TiffHandlerImpl : public TiffHandler {
 public:
  TiffHandlerImpl();
  ~TiffHandlerImpl() override;

  auto Write(const Image* image, const std::filesystem::path& log_path, uint32_t x_offset, uint32_t y_offset)
      -> void override;

  auto SetJointGeometry(const std::string& joint_geometry_yaml) -> void override {
    joint_geometry_yaml_ = joint_geometry_yaml;
  };
  auto SetScannerCalibration(const std::string& scanner_calib_yaml) -> void override {
    scanner_calib_yaml_ = scanner_calib_yaml;
  };

 private:
  std::string joint_geometry_yaml_;
  std::string scanner_calib_yaml_;
};
};  // namespace scanner::image
