#pragma once

#include <Eigen/Dense>

#include <optional>

#include "bead_control/src/weld_position_data_storage.h"
#include "common/containers/position_buffer.h"
#include "common/groove/groove.h"

namespace bead_control {

class GrooveFit {
 public:
  enum class Type { POLYNOMIAL, FOURIER };

  GrooveFit(const WeldPositionDataStorage::Slice& slice, Type type, int order, uint32_t max_samples)
      : type_(type), order_(order), coefficients_(common::groove::ABW_POINTS) {
    max_samples_ = max_samples == 0 ? slice.Size() : std::min(static_cast<size_t>(max_samples), slice.Size());
    uint32_t const samples = static_cast<uint32_t>(max_samples_);

    Eigen::MatrixXd aa(samples, order_ + 1);
    Eigen::MatrixXd bh(samples, common::groove::ABW_POINTS);
    Eigen::MatrixXd bv(samples, common::groove::ABW_POINTS);

    buildMatrix(slice, aa, bh, bv);

    auto const bh_ = aa.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(bh);
    auto const bv_ = aa.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(bv);

    coefficients_ = (bh_.array().square().colwise().sum().sqrt().matrix() +
                     bv_.array().square().colwise().sum().sqrt().matrix())
                        .transpose();
  }

  auto Fit(double pos) -> common::groove::Groove {
    auto groove = common::groove::Groove();
    for (auto abw_point = 0; abw_point < common::groove::ABW_POINTS; ++abw_point) {
      switch (type_) {
        case Type::POLYNOMIAL: {
          groove[abw_point] = {.horizontal = Polynomial(pos, bh_.col(abw_point)),
                               .vertical   = Polynomial(pos, bv_.col(abw_point))};
          break;
        }
        case Type::FOURIER: {
          groove[abw_point] = {.horizontal = Fourier(pos, bh_.col(abw_point)),
                               .vertical   = Fourier(pos, bv_.col(abw_point))};
          break;
        }
      }
    }
    return groove;
  }

  auto Coefficients() const -> Eigen::MatrixXd { return coefficients_; }

 private:
  size_t max_samples_{};
  Type type_;
  int order_;
  Eigen::MatrixXd coefficients_;
  Eigen::MatrixXd bh_;
  Eigen::MatrixXd bv_;

  static void buildMatrix(const WeldPositionDataStorage::Slice& slice, Eigen::MatrixXd& aa, Eigen::MatrixXd& bh,
                          Eigen::MatrixXd& bv);

  static auto Polynomial(double pos, Eigen::VectorXd coeff) -> double;
  static auto Fourier(double pos, Eigen::VectorXd coeff) -> double;
};

}  // namespace bead_control
