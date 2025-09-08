#pragma once

#include <cstdint>
#include <vector>

namespace controller {

class WeldAxisFilter {
 public:
  virtual ~WeldAxisFilter() = default;

  virtual auto ProcessSignal(float raw_value) -> float = 0;
  virtual void ClearSignalBuffer()                     = 0;
};

class WeldAxisFilterMedianImpl : public WeldAxisFilter {
 public:
  explicit WeldAxisFilterMedianImpl(uint8_t window_size);
  ~WeldAxisFilterMedianImpl() override = default;

  WeldAxisFilterMedianImpl(WeldAxisFilterMedianImpl&)                     = delete;
  auto operator=(WeldAxisFilterMedianImpl&) -> WeldAxisFilterMedianImpl&  = delete;
  WeldAxisFilterMedianImpl(WeldAxisFilterMedianImpl&&)                    = delete;
  auto operator=(WeldAxisFilterMedianImpl&&) -> WeldAxisFilterMedianImpl& = delete;

  auto ProcessSignal(float raw_value) -> float override;
  void ClearSignalBuffer() override;
  auto GetWindowSize() -> uint8_t;

 private:
  uint8_t window_size_;
  std::vector<float> continuous_signal_;
  std::vector<float> arrange_median_;

  void FindMedian();
};

}  // namespace controller
