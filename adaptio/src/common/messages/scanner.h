#pragma once

#include <chrono>
#include <cstdint>

#include "common/groove/point.h"

namespace common::msg::scanner {

const uint32_t SCANNER_BASE_ID = 0x04000000;

const uint32_t GROOVE_ARRAY_SIZE  = 7;
const uint32_t PROFILE_ARRAY_SIZE = 100;

// Values in millimeter
struct Coordinate {
  double x;
  double y;
};

enum class ImageLoggerMode {
  OFF,
  DIRECT,
  BUFFERED,
  ON_ERROR,
};

struct JointGeometry {
  double upper_joint_width_mm;
  double groove_depth_mm;
  double left_joint_angle_rad;
  double right_joint_angle_rad;
  double left_max_surface_angle_rad;
  double right_max_surface_angle_rad;
  struct {
    double upper_width_mm;
    double surface_angle_rad;
    double wall_angle_rad;
  } tolerance;
};

enum class ScannerSensitivity {
  NORMAL = 0,
  HIGH   = 1,
};

enum class SliceConfidence {
  NO     = 0,
  LOW    = 1,
  MEDIUM = 2,
  HIGH   = 3,
};

struct Start {
  enum class Metadata : uint32_t { MESSAGE_ID = SCANNER_BASE_ID + 1 };
  JointGeometry joint_geometry;
  enum ScannerSensitivity sensitivity;
  std::chrono::milliseconds interval;
};

struct StartRsp {
  enum class Metadata : uint32_t { MESSAGE_ID = SCANNER_BASE_ID + 2 };
  bool success;
};

struct Update {
  enum class Metadata : uint32_t { MESSAGE_ID = SCANNER_BASE_ID + 3 };
  JointGeometry joint_geometry;
  double abw0_horizontal;
  double abw6_horizontal;
};

struct Stop {
  enum class Metadata : uint32_t { MESSAGE_ID = SCANNER_BASE_ID + 5 };
};

struct SliceData {
  enum class Metadata : uint32_t { MESSAGE_ID = SCANNER_BASE_ID + 6 };
  Coordinate groove[GROOVE_ARRAY_SIZE];
  std::array<Coordinate, PROFILE_ARRAY_SIZE> profile;
  enum SliceConfidence confidence;
  std::uint64_t time_stamp;  // Milliseconds since Epoch
  double groove_area{0.};
};

struct FlushImageBuffer {
  enum class Metadata : uint32_t { MESSAGE_ID = SCANNER_BASE_ID + 7 };
};

struct StopRsp {
  enum class Metadata : uint32_t { MESSAGE_ID = SCANNER_BASE_ID + 8 };
  bool success;
};

struct ImageLoggingData {
  enum class Metadata : uint32_t { MESSAGE_ID = SCANNER_BASE_ID + 9 };
  ImageLoggerMode mode;
  uint32_t sample_rate;
  uint32_t depth;
  std::chrono::milliseconds on_error_interval;
  char path[512];
};

}  // namespace common::msg::scanner
