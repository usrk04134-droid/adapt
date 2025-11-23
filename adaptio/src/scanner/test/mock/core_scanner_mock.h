#pragma once

#include <trompeloeil.hpp>

#include "scanner/core/src/scanner.h"

// NOLINTBEGIN(*-magic-numbers, *-optional-access)

class CoreScannerMock : public trompeloeil::mock_interface<scanner::Scanner> {
  IMPLEMENT_MOCK1(Start);
  IMPLEMENT_MOCK0(Stop);
  IMPLEMENT_MOCK0(Update);
  void ImageGrabbed(std::unique_ptr<scanner::image::Image> image) override {};
  void UpdateJointApproximation(const scanner::joint_model::JointProperties& properties,
                                const std::tuple<double, double>& abw0_abw6_horizontal) override {};
  size_t CountOfReceivedImages() override { return 0; };
};

// NOLINTEND(*-magic-numbers, *-optional-access)
