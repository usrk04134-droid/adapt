#pragma once

namespace image_logging {

class ImageLoggingManager {
 public:
  virtual ~ImageLoggingManager() = default;

  virtual void Flush() = 0;
};

}  // namespace image_logging
