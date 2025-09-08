#pragma once

#include <boost/circular_buffer.hpp>
#include <cmath>
#include <vector>

namespace common::filters {

class GaussianFilter {
 public:
  GaussianFilter(int kernel_size, double sigma) : buffer_(kernel_size), kernel_size_(kernel_size), sigma_(sigma) {
    assert(kernel_size > 0 && "Kernel size must be positive number");
    assert(kernel_size % 2 != 0 && "Kernel size must be odd number");
    assert(sigma > 0 && "Sigma must be positive");

    kernel_.resize(kernel_size_);
    auto sum             = 0.0;
    auto const half_size = kernel_size_ / 2;

    for (int i = 0; i < kernel_size_; ++i) {
      auto const xval = static_cast<double>(i - half_size);
      kernel_[i]  = std::exp(-(xval * xval) / (2.0 * sigma_ * sigma_)) / (std::sqrt(2.0 * std::numbers::pi) * sigma_);
      sum        += kernel_[i];
    }

    for (int i = 0; i < kernel_size_; ++i) {
      kernel_[i] /= sum;
    }
  }

  auto Update(double input_value) -> double {
    buffer_.push_back(input_value);

    if (buffer_.full()) {
      double smoothed_value = 0.0;
      for (int i = 0; i < kernel_size_; ++i) {
        smoothed_value += buffer_[i] * kernel_[i];
      }
      return smoothed_value;
    }
    return input_value;
  }

  void Reset() { buffer_.clear(); }

  void Fill(double value) {
    buffer_.clear();
    buffer_.assign(kernel_size_, value);
  }

 private:
  boost::circular_buffer<double> buffer_;
  std::vector<double> kernel_;
  int kernel_size_;
  double sigma_;
};

}  // namespace common::filters
