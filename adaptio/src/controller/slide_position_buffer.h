#pragma once

#include <boost/circular_buffer.hpp>
#include <cstdint>

namespace controller {

class SlidePositionBuffer {
 public:
  virtual ~SlidePositionBuffer() = default;

  virtual auto StorePosition(double position, uint64_t time_stamp) -> bool = 0;
  virtual auto GetPosition(uint64_t time_stamp) -> double                  = 0;
  virtual auto Capacity() -> uint64_t                                      = 0;
};

struct SlidePositionEntry {
  double position;
  uint64_t time_stamp;
};

class SlidePositionBufferImpl final : public SlidePositionBuffer {
 public:
  SlidePositionBufferImpl();

  auto StorePosition(double position, uint64_t time_stamp) -> bool override;
  auto GetPosition(uint64_t time_stamp) -> double override;
  auto Capacity() -> uint64_t override;

 private:
  boost::circular_buffer<SlidePositionEntry> buffer_;
};

}  // namespace controller
