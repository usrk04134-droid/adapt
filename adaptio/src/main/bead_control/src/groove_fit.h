#pragma once

#include <cassert>
#include <Eigen/Eigen>
#include <vector>

#include "bead_control/src/weld_position_data_buffer.h"
#include "common/groove/groove.h"

namespace bead_control {
class GrooveFit {
 public:
  enum class Type {
    POLYNOMIAL,
    FOURIER,
  };

  GrooveFit(const WeldPositionDataBuffer& buffer, Type type, int order, uint32_t max_samples)
      : type_(type), order_(order), coefficients_(common::ABW_POINTS) {
    auto const sz      = buffer.FilledSlots();
    auto const samples = max_samples == 0 || sz <= max_samples ? sz : max_samples;
    auto const step    = static_cast<double>(sz) / samples;

    auto const num_coefficients = type_ == Type::POLYNOMIAL ? order_ + 1 : (2 * order) + 1;
    Eigen::MatrixXd aa(samples, num_coefficients);
    Eigen::MatrixXd bh(samples, common::ABW_POINTS);
    Eigen::MatrixXd bv(samples, common::ABW_POINTS);
    for (auto i = 0; i < samples; ++i) {
      auto maybe_weld_data = buffer.Get(i * step);

      if (!maybe_weld_data.has_value()) {
        continue;
      }

      auto const& [pos, data] = maybe_weld_data.value();

      if (type_ == Type::POLYNOMIAL) {
        for (auto j = 0; j < order_; ++j) {
          aa(i, j) = std::pow(pos, order_ - j);
        }
        aa(i, order_) = 1.0;
      } else {
        for (auto k = 1; k <= order_; ++k) {
          aa(i, (2L * k) - 1) = std::cos(k * pos);
          aa(i, (2L * k))     = std::sin(k * pos);
        }
        aa(i, 0) = 1.0;
      }

      for (auto abw_point = 0; abw_point < common::ABW_POINTS; ++abw_point) {
        bh(i, abw_point) = data.groove[abw_point].horizontal;
        bv(i, abw_point) = data.groove[abw_point].vertical;
      }

      for (auto abw_point = 0; abw_point < common::ABW_POINTS; ++abw_point) {
        coefficients_[abw_point] = {
            .horizontal = aa.colPivHouseholderQr().solve(bh.col(abw_point)),
            .vertical   = aa.colPivHouseholderQr().solve(bv.col(abw_point)),

            //  Eigen::BDCSVD is an alternative Eigen::ColPivHouseholderQR that is more robust but much slower
            // .horizontal = aa.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(bh.col(abw_point)),
            // .vertical   = aa.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(bv.col(abw_point)),
        };
      }
    }
  }

  auto Fit(double pos) -> common::Groove {
    auto groove = common::Groove();
    for (auto abw_point = 0; abw_point < common::ABW_POINTS; ++abw_point) {
      switch (type_) {
        case Type::POLYNOMIAL:
          for (auto i = 0; i <= order_; ++i) {
            groove[abw_point] += {
                .horizontal = coefficients_[abw_point].horizontal[i] * std::pow(pos, order_ - i),
                .vertical   = coefficients_[abw_point].vertical[i] * std::pow(pos, order_ - i),
            };
          }
          break;

        case Type::FOURIER:
          groove[abw_point].horizontal = coefficients_[abw_point].horizontal(0);
          groove[abw_point].vertical   = coefficients_[abw_point].vertical(0);
          for (int k = 1; k <= order_; ++k) {
            groove[abw_point] += {
                .horizontal = (coefficients_[abw_point].horizontal((2L * k) - 1) * std::cos(k * pos)) +
                              (coefficients_[abw_point].horizontal(2L * k) * std::sin(k * pos)),
                .vertical = (coefficients_[abw_point].vertical((2L * k) - 1) * std::cos(k * pos)) +
                            (coefficients_[abw_point].vertical(2L * k) * std::sin(k * pos)),
            };
          }
          break;
      }
    }

    return groove;
  }

 private:
  Type type_{Type::POLYNOMIAL};
  int order_{};

  struct Coefficients {
    Eigen::VectorXd horizontal;
    Eigen::VectorXd vertical;
  };
  std::vector<Coefficients> coefficients_;
};

}  // namespace bead_control
