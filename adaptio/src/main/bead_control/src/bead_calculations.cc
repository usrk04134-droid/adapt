
#include "bead_calculations.h"

#include <cassert>
#include <cmath>
#include <numbers>
#include <numeric>
#include <vector>

#include "common/groove/groove.h"
#include "common/groove/math.h"
#include "common/groove/point.h"
#include "common/logging/application_log.h"

namespace {
auto const BEAD_HEIGHT_FACTOR_MIN = 0.6;
auto const BEAD_HEIGHT_FACTOR_MAX = 1.0;

// Calculate an coordinate on a line between two other coordinates.
// The height is from the lower coordinate on the line.
// The returned coordinate is the intersection between the horizontal
// line from height and the line between the two coordinates
auto WallCoordinate(common::Point lower_coord, common::Point upper_coord, double height) -> common::Point {
  // Should not happen but avoid division with zero
  if (upper_coord.vertical == lower_coord.vertical || upper_coord.horizontal == lower_coord.horizontal) {
    return lower_coord;
  }

  auto const k_slope =
      (upper_coord.vertical - lower_coord.vertical) / (lower_coord.horizontal - upper_coord.horizontal);

  return {.horizontal = lower_coord.horizontal - (height / k_slope), .vertical = lower_coord.vertical + height};
}
};  // namespace

namespace bead_control {

auto BeadCalc::MeanLayerArea(const common::Groove& groove, double left_bead_area, double right_bead_area,
                             double step_up_value) -> double {
  // map step up value: 0.0 - 1.0 to bead height factor:  0.6 - 1.0
  auto bead_height_factor =
      BEAD_HEIGHT_FACTOR_MIN + ((BEAD_HEIGHT_FACTOR_MAX - BEAD_HEIGHT_FACTOR_MIN) * step_up_value);

  auto const left_bead_height  = bead_height_factor * sqrt(2 * left_bead_area / std::numbers::pi);
  auto const right_bead_height = bead_height_factor * sqrt(2 * right_bead_area / std::numbers::pi);

  LOG_INFO("Calculated Left bead height: {:.5f} Right bead height: {:.5f}", left_bead_height, right_bead_height);

  return common::PolygonArea(
      {WallCoordinate(groove[common::ABW_LOWER_LEFT], groove[common::ABW_UPPER_LEFT], left_bead_height),
       groove[common::ABW_LOWER_LEFT], groove[common::ABW_LOWER_RIGHT],
       WallCoordinate(groove[common::ABW_LOWER_RIGHT], groove[common::ABW_UPPER_RIGHT], right_bead_height)});
}

auto BeadCalc::BeadArea(double wire_lin_velocity, double wire_diameter, double weld_object_lin_velocity) -> double {
  assert(weld_object_lin_velocity > 0.);
  return std::numbers::pi * wire_lin_velocity * std::pow((wire_diameter / 2.), 2.) / weld_object_lin_velocity;
}

auto BeadCalc::BeadSliceAreaRatio(const common::Groove& groove, int bead, int beads) -> double {
  if (beads < 2 || bead < 1 || bead > beads) {
    LOG_ERROR("Invalid input bead/beads: {}/{}", bead, beads);
    return 1.;
  }

  auto const abw_points       = common::ABW_POINTS;
  auto const abw_point_slices = abw_points - 3;
  auto const fbeads           = static_cast<double>(beads);
  auto const slice_size       = abw_point_slices / fbeads;
  auto const right_pos        = bead * slice_size;
  auto const left_pos         = (bead - 1) * slice_size;

  // vector holding bead slice vetrticies in clock-wise order
  auto vec = std::vector<common::Point>();

  auto const abw_ul = groove[common::ABW_UPPER_LEFT];
  auto const abw_ur = groove[common::ABW_UPPER_RIGHT];

  // top-left vertex
  vec.push_back(common::Point{
      .horizontal = std::lerp(abw_ul.horizontal, abw_ur.horizontal, (bead - 1) / fbeads),
      .vertical   = std::lerp(abw_ul.vertical, abw_ur.vertical, (bead - 1) / fbeads),
  });

  // top-right vertex
  vec.push_back(common::Point{
      .horizontal = std::lerp(abw_ul.horizontal, abw_ur.horizontal, bead / fbeads),
      .vertical   = std::lerp(abw_ul.vertical, abw_ur.vertical, bead / fbeads),
  });

  // Store bottom groove verticies right-to-left (clock-wise).
  // The verticies stored are iterpolated between nearby ABW points.
  const int abw_slice_start_indx = static_cast<int>(std::ceil(right_pos)) - 1;
  for (int abw_slice = abw_slice_start_indx; abw_slice >= std::floor(left_pos); --abw_slice) {
    // ABW slice right and left position
    auto const right = std::fmin(right_pos, abw_slice + 1) - abw_slice;
    auto const left  = std::fmax(left_pos, abw_slice) - abw_slice;

    if (abw_slice == abw_slice_start_indx) {
      vec.push_back(common::Point{
          .horizontal = std::lerp(groove[abw_slice + 1].horizontal, groove[abw_slice + 2].horizontal, right),
          .vertical   = std::lerp(groove[abw_slice + 1].vertical, groove[abw_slice + 2].vertical, right),
      });
    }

    vec.push_back(common::Point{
        .horizontal = std::lerp(groove[abw_slice + 1].horizontal, groove[abw_slice + 2].horizontal, left),
        .vertical   = std::lerp(groove[abw_slice + 1].vertical, groove[abw_slice + 2].vertical, left),
    });
  }

  auto const groove_area     = groove.Area();
  auto const bead_slice_area = common::PolygonArea(vec);

  /*LOG_TRACE("area groove/slice: {} / {} ({:.2f}%)", groove_area, bead_slice_area,*/
  /*          bead_slice_area / groove_area * 100. * fbeads);*/

  return bead_slice_area / groove_area * fbeads;
}

auto BeadCalc::BeadPositionAdjustment(const common::Groove& groove, double bead_pos, double k_gain) -> double {
  auto const k_top        = groove.TopSlope();
  auto const k_bot        = groove.BottomSlope();
  auto const new_bead_pos = k_bot > k_top ? std::pow(bead_pos, 1.0 - ((k_top - k_bot) * k_gain))
                                          : 1.0 - std::pow(1.0 - bead_pos, 1.0 + ((k_top - k_bot) * k_gain));

  /*LOG_TRACE("k-top/k-bot: {:.3f} / {:.3f} bead-pos / new-bead-pos: {:.2f} / {:.2f}", k_top, k_bot, bead_pos,*/
  /*          new_bead_pos);*/

  return new_bead_pos;
}

auto BeadCalc::MeanLayerTopWidth(const common::Groove& groove, double left_bead_area, double right_bead_area,
                                 double step_up_value) -> double {
  auto bead_height_factor =
      BEAD_HEIGHT_FACTOR_MIN + ((BEAD_HEIGHT_FACTOR_MAX - BEAD_HEIGHT_FACTOR_MIN) * step_up_value);

  auto const left_bead_height  = bead_height_factor * std::sqrt(2 * left_bead_area / std::numbers::pi);
  auto const right_bead_height = bead_height_factor * std::sqrt(2 * right_bead_area / std::numbers::pi);

  LOG_DEBUG("Measured depth of previous layer: left {:.5f}, right {:.5f}", groove.LeftDepth(), groove.RightDepth());
  LOG_DEBUG("Calculated Left bead height: {:.5f} Right bead height: {:.5f}", left_bead_height, right_bead_height);

  auto const left_top =
      WallCoordinate(groove[common::ABW_LOWER_LEFT], groove[common::ABW_UPPER_LEFT], left_bead_height);
  auto const right_top =
      WallCoordinate(groove[common::ABW_LOWER_RIGHT], groove[common::ABW_UPPER_RIGHT], right_bead_height);

  auto const top_width = std::fabs((right_top - left_top).horizontal);
  LOG_DEBUG("Measured bottom layer width: {:.5f}", groove.BottomWidth());
  LOG_DEBUG("Calculated layer top width: {:.5f}", top_width);
  return top_width;
}

}  // namespace bead_control
