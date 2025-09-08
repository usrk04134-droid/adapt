#pragma once

#include <boost/circular_buffer.hpp>
#include <cstdint>

namespace controller {

struct WeldAxisPosition {
  float position;
  float velocity;
  uint64_t turn_counter;
};

struct PositionElement {
  WeldAxisPosition weld_axis;
  uint64_t time_stamp;
};

class WeldAxisBuffer {
 public:
  virtual ~WeldAxisBuffer() = default;

  virtual auto StorePosition(float position, float velocity, uint64_t time_stamp) -> bool = 0;
  virtual auto GetPosition(uint64_t time_stamp) -> WeldAxisPosition                       = 0;
  virtual auto Capacity() -> uint64_t                                                     = 0;
};

class WeldAxisBufferImpl final : public WeldAxisBuffer {
 public:
  WeldAxisBufferImpl();
  ~WeldAxisBufferImpl() override = default;

  WeldAxisBufferImpl(WeldAxisBufferImpl&)                     = delete;
  auto operator=(WeldAxisBufferImpl&) -> WeldAxisBufferImpl&  = delete;
  WeldAxisBufferImpl(WeldAxisBufferImpl&&)                    = delete;
  auto operator=(WeldAxisBufferImpl&&) -> WeldAxisBufferImpl& = delete;

  auto StorePosition(float position, float velocity, uint64_t time_stamp) -> bool override;
  auto GetPosition(uint64_t time_stamp) -> WeldAxisPosition override;
  auto Capacity() -> uint64_t override;

 private:
  boost::circular_buffer<PositionElement> buffer_;
  uint32_t turn_;
};

}  // namespace controller
