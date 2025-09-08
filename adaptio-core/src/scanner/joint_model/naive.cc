#include "core/scanner/joint_model/naive.h"

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <Eigen/src/Core/util/IndexedViewHelper.h>
#include <math.h>

#include <algorithm>
#include <array>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <expected>
#include <limits>
#include <opencv2/core/eigen.hpp>
#include <optional>
#include <ranges>
#include <tuple>
#include <utility>
#include <vector>

#include "core/image/camera_model.h"
#include "core/image/image.h"
#include "core/image/image_types.h"
#include "core/logging/application_log.h"
#include "core/math/centroids.h"
#include "core/math/filter.h"
#include "core/math/lin_interp.h"
#include "core/math/value.h"
#include "core/scanner/joint_model.h"
#include "core/scanner/scanner_configuration.h"
#include "image/image_utilities.h"

using core::image::ImageUtility;
using core::image::PlaneCoordinates;
using core::image::RawImageData;
using core::scanner::ABWPoints;
using core::scanner::JointModel;
using core::scanner::JointProfile;
using core::scanner::JointProperties;
using core::scanner::LineSegment;
using core::scanner::Point;
using core::scanner::joint_model::Naive;

// Model constants
const int GROOVE_BUFFER_DEPTH = 20;
const double SP               = 0.01;  // Scaling in meters.

Naive::Naive(const JointProperties& properties, const core::scanner::ScannerConfigurationData& config_data,
             core::image::CameraModelPtr camera_model)
    : JointModel(properties, std::move(camera_model)),
      groove_wall_height_buffer_(GROOVE_BUFFER_DEPTH),
      groove_wall_angles_buffer_(GROOVE_BUFFER_DEPTH),
      wedge_buffer_(GROOVE_BUFFER_DEPTH),
      gray_minimum_top(config_data.gray_minimum_top),
      gray_minimum_wall(config_data.gray_minimum_wall),
      gray_minimum_bottom(config_data.gray_minimum_bottom) {}

auto Naive::Parse(core::image::Image& image, std::optional<JointProfile> median_profile,
                  std::optional<JointProperties> empty_joint_properties)
    -> std::expected<std::tuple<JointProfile, core::image::WorkspaceCoordinates, uint64_t, uint64_t>,
                     JointModelErrorCode> {
  JointProfile profile;
  uint64_t num_walls_found;

  core::image::WorkspaceCoordinates joint;
  core::image::WorkspaceCoordinates centroids_wcs;

  uint64_t processing_time = 0;
  auto start               = std::clock();

  std::optional<Eigen::Matrix<double, 2, Eigen::Dynamic, Eigen::RowMajor>> maybe_centroids;
  maybe_centroids =
      core::math::GetCentroidsByColumn(image.Data(), core::math::CentroidSearchDirection::Reversed, gray_minimum_top);

  if (!maybe_centroids.has_value()) {
    LOG_WARNING("Not able to get centroids for image.");
    return std::unexpected(JointModelErrorCode::SURFACE_NOT_FOUND);
  }

  auto centroids             = maybe_centroids.value();
  auto crop_start            = image.GetVerticalCropStart();
  auto maybe_wcs_coordinates = camera_model_->ImageToWorkspace(centroids, crop_start);

  if (!maybe_wcs_coordinates.has_value()) {
    return std::unexpected(JointModelErrorCode::SURFACE_NOT_FOUND);
  }

  joint         = maybe_wcs_coordinates.value();
  centroids_wcs = joint;

  LOG_TRACE("Joint width (mm): {} - {} = {}", joint(0, Eigen::last) * 1e3, joint(0, 0) * 1e3,
            (joint(0, Eigen::last) - joint(0, 0)) * 1e3);

  const double min = static_cast<double>(joint.block(1, 0, 1, joint.cols()).minCoeff());
  const double max = static_cast<double>(joint.block(1, 0, 1, joint.cols()).maxCoeff());

  LOG_TRACE("Joint height (mm): {} - {} = {}", max * 1e3, min * 1e3, (max - min) * 1e3);

  auto [maybe_left_surface, maybe_right_surface] = SurfaceLines(joint.row(0), joint.row(1));

  if (!maybe_left_surface.has_value() || !maybe_right_surface.has_value()) {
    // return {JointModelErrorCode::SURFACE_NOT_FOUND, centroids_wcs, processing_time, 0};
    return std::unexpected(JointModelErrorCode::SURFACE_NOT_FOUND);
  }
  auto left_surface  = maybe_left_surface.value();
  auto right_surface = maybe_right_surface.value();

  auto [left_groove_depth, right_groove_depth] = GetMedianGrooveDepths();

  if (welding_mode_ == WELD_MODE_REGULAR) {
    auto maybe_new_left_surface  = GetImprovedSurfaceLine(joint, left_surface, false, 0.0003);
    auto maybe_new_right_surface = GetImprovedSurfaceLine(joint, right_surface, true, 0.0003);

    if (!maybe_new_left_surface.has_value() || !maybe_new_right_surface.has_value()) {
      return std::unexpected(JointModelErrorCode::SURFACE_NOT_FOUND);
    }

    left_surface  = maybe_new_left_surface.value();
    right_surface = maybe_new_right_surface.value();
  }

  std::tuple<double, double, double> left_surface_edges = {
      left_surface.x_limits.max, left_surface.x_limits.max * left_surface.k + left_surface.m, 0.0};
  std::tuple<double, double, double> right_surface_edges = {
      right_surface.x_limits.min, right_surface.x_limits.min * right_surface.k + right_surface.m, 0.0};

  auto wall_coords_laser = GetWallCoordsLaserPlane(left_surface, right_surface, left_surface_edges, right_surface_edges,
                                                   left_groove_depth, right_groove_depth);
  auto wall_coords_image = TransformCoordsImagePlane(wall_coords_laser, crop_start);

  auto maybe_left_groove_wall =
      GetWallImage(wall_coords_image, image.Data(), false)
          .and_then([this, left_surface, left_surface_edges,
                     crop_start](std::tuple<const image::RawImageData, std::tuple<int, int>> i) {
            auto [image, offset] = i;
            return GetWallCentroids(left_surface, left_surface_edges, image, offset, crop_start, false);
          })
          .and_then([this](core::image::WorkspaceCoordinates centroids) { return GrooveLine(centroids); });

  auto maybe_right_groove_wall =
      GetWallImage(wall_coords_image, image.Data(), true)
          .and_then([this, right_surface, right_surface_edges,
                     crop_start](std::tuple<const image::RawImageData, std::tuple<int, int>> i) {
            auto [image, offset] = i;
            return GetWallCentroids(right_surface, right_surface_edges, image, offset, crop_start, true);
          })
          .and_then([this](core::image::WorkspaceCoordinates centroids) { return GrooveLine(centroids); });

  bool found_both_walls = maybe_left_groove_wall.has_value() && maybe_right_groove_wall.has_value();
  if (!found_both_walls) {
    num_walls_found = (maybe_left_groove_wall.has_value() || maybe_right_groove_wall.has_value()) ? 1 : 0;
  } else {
    num_walls_found = 2;
  }

  if (welding_mode_ == WELD_MODE_REGULAR && num_walls_found < 2) {
    consecutive_frames_without_finding_both_walls++;
    if (consecutive_frames_without_finding_both_walls > 5) {
      welding_mode_ = WELD_MODE_CAP;
    } else {
      return std::unexpected(JointModelErrorCode::GROOVE_WALL_CENTROIDS_NOT_FOUND);
    }
  } else {
    consecutive_frames_without_finding_both_walls = 0;
  }

  std::optional<ABWPoints> maybe_abw_points;

  if (welding_mode_ == WELD_MODE_REGULAR || found_both_walls) {
    auto left_groove_wall  = maybe_left_groove_wall.value();
    auto right_groove_wall = maybe_right_groove_wall.value();

    auto maybe_pwl_wedge = FitGrooveLine(left_surface, right_surface, left_groove_wall, right_groove_wall,
                                         left_surface_edges, right_surface_edges);
    if (!maybe_pwl_wedge.has_value()) {
      return std::unexpected(JointModelErrorCode::WEDGE_FIT_FAILED);
    }

    auto pwl_wedge = maybe_pwl_wedge.value();
    wedge_buffer_.push_back(pwl_wedge);

    auto joint_bottom = GetBottomCentroids(image, left_groove_wall, right_groove_wall, pwl_wedge);
    if (!joint_bottom.has_value()) {
      return std::unexpected(JointModelErrorCode::GROOVE_BOTTOM_NOT_FOUND);
    }

    profile.area = GetJointArea(joint_bottom.value(), pwl_wedge);

    auto groove_bottom = GrooveBottomLine(joint_bottom.value().row(0), joint_bottom.value().row(1), pwl_wedge);
    if (!groove_bottom.has_value()) {
      return std::unexpected(JointModelErrorCode::GROOVE_BOTTOM_NOT_FOUND);
    }
    maybe_abw_points = ExtractABWPoints(pwl_wedge, groove_bottom.value());
  } else {
    if (wedge_buffer_.size() == 0) {
      return std::unexpected(JointModelErrorCode::MISSING_WEDGE_HISTORY);
    }
    profile.area = 0.0;  // TODO: calculate area from ABW points.

    maybe_abw_points = ExtractABWPointsCapWeldingMode(maybe_left_groove_wall, maybe_right_groove_wall, left_surface,
                                                      right_surface, joint);
  }

  if (!maybe_abw_points.has_value()) {
    return std::unexpected(JointModelErrorCode::INVALID_SNAKE);
  }

  auto abw_points = maybe_abw_points.value();

  previous_left_edge  = abw_points[0].x;
  previous_right_edge = abw_points[6].x;

  auto left_groove_wall_height  = abw_points[0].y - abw_points[1].y;
  auto right_groove_wall_height = abw_points[6].y - abw_points[5].y;

  LOG_TRACE("Push groove depth: {} {}", left_groove_wall_height, right_groove_wall_height);
  groove_wall_height_buffer_.push_back({left_groove_wall_height, right_groove_wall_height});

  profile.points = abw_points;

  auto maybe_abw_points_in_image_coordinates =
      camera_model_->WorkspaceToImage(ABWPointsToMatrix(abw_points), crop_start);

  if (maybe_abw_points_in_image_coordinates.has_value()) {
    auto abw_points_in_image_coordinates = maybe_abw_points_in_image_coordinates.value();
    auto bottom_pixel       = static_cast<int>(abw_points_in_image_coordinates.block(1, 0, 1, 7).maxCoeff());
    auto top_pixel          = static_cast<int>(abw_points_in_image_coordinates.block(1, 0, 1, 7).minCoeff());
    profile.vertical_limits = {top_pixel, bottom_pixel};
  }

  processing_time = (std::clock() - start) * 1000 / CLOCKS_PER_SEC;

  return std::make_tuple(profile, centroids_wcs, processing_time, num_walls_found);
}

auto Naive::GetBottomCentroids(const core::image::Image& image, const LineSegment& left_groove_wall,
                               const LineSegment& right_groove_wall, const PwlWedge& pwl_wedge)
    -> std::optional<image::WorkspaceCoordinates> {
  auto y_top    = std::max(left_groove_wall.y_limits.min, right_groove_wall.y_limits.min) + 10.e-3;
  auto y_bottom = (pwl_wedge.y1t * SP - properties_.groove_depth * 1.0e-3 - 2.e-3);

  // Get coordinates in laser plane around groove bottom
  core::image::WorkspaceCoordinates mask_points_wcs(3, 2);
  mask_points_wcs << pwl_wedge.x_left, pwl_wedge.x_right, y_top, y_bottom, 0., 0.;

  // Transform coordinates to image plane
  auto maybe_img = camera_model_->WorkspaceToImage(mask_points_wcs, image.GetVerticalCropStart());

  if (!maybe_img.has_value()) {
    LOG_ERROR("Not able to transform workspace to image");
    return std::nullopt;
  }

  auto img = maybe_img.value();

  auto c_left  = int(round(img(0, 0)));
  auto r_left  = int(round(img(1, 0)));
  auto c_right = int(round(img(0, 1)));

  auto maybe_bottom_image =
      ImageUtility::CropImage(image.Data(), r_left, c_left, image.Data().rows() - r_left, c_right - c_left);

  if (!maybe_bottom_image.has_value()) {
    LOG_ERROR("Not able to crop bottom image.");
    return {};
  }

  auto maybe_centroids = core::math::GetCentroidsByColumn(
      maybe_bottom_image.value(), core::math::CentroidSearchDirection::Reversed, gray_minimum_bottom);

  if (!maybe_centroids.has_value()) {
    LOG_TRACE("Not able to get centroids for groove bottom.");
    return std::nullopt;
  }

  // Do we need at least 30 centroids?
  if (maybe_centroids.value().cols() < 30) {
    LOG_TRACE("Too few centroids for groove bottom.");
    return std::nullopt;
  }

  auto centroids = maybe_centroids.value();

  // Set correct coordinates in cropped image
  centroids.row(0).array() += (double)c_left;
  centroids.row(1).array() += (double)r_left;

  auto maybe_bottom_joint = camera_model_->ImageToWorkspace(centroids, image.GetVerticalCropStart());

  if (!maybe_bottom_joint.has_value()) {
    LOG_ERROR("Not able to transform image to workspace");
    return std::nullopt;
  }

  return maybe_bottom_joint.value();
}

auto Naive::GetJointArea(core::image::WorkspaceCoordinates centroids, const PwlWedge& pwl_wedge) -> double {
  // Filter centroids to only include those within the wedge.
  auto [x_coords, y_coords] = ExcludeCentroidsOutsideWedge(pwl_wedge, centroids.row(0), centroids.row(1));

  // Add the top corners of the wedge.
  x_coords.push_back(pwl_wedge.x3t * SP);
  y_coords.push_back(pwl_wedge.y3t * SP);

  x_coords.push_back(pwl_wedge.x1t * SP);
  y_coords.push_back(pwl_wedge.y1t * SP);

  // Calculate the area with the shoelace formula
  double area = 0.0;
  for (int i = 0; i < x_coords.size(); i++) {
    int j  = (i == 0) ? (x_coords.size() - 1) : (i - 1);
    area  += x_coords[j] * y_coords[i] - x_coords[i] * y_coords[j];
  }

  return area * 0.5;
}

auto Naive::GetWallCentroids(const LineSegment& surface, const std::tuple<double, double, double>& surface_edges,
                             const RawImageData& image, std::tuple<int, int> offset, int extra_vertical_offset,
                             bool right_wall) -> std::optional<core::image::WorkspaceCoordinates> {
  auto direction =
      right_wall ? core::math::CentroidSearchDirection::Reversed : core::math::CentroidSearchDirection::Normal;
  auto [x_offset, y_offset] = offset;
  return core::math::GetCentroidsByRow(image, direction, gray_minimum_wall)
      .and_then([this, right_wall, x_offset, y_offset, extra_vertical_offset](
                    core::image::PlaneCoordinates centroids) -> std::optional<core::image::WorkspaceCoordinates> {
        LOG_TRACE("{} wall centroids", centroids.cols());
        centroids.row(0).array() += x_offset;
        centroids.row(1).array() += y_offset;
        auto c                    = camera_model_->ImageToWorkspace(centroids, extra_vertical_offset);
        if (c.has_value()) {
          auto centroids = c.value();
          if (right_wall) {
            centroids.row(0).reverseInPlace();
            centroids.row(1).reverseInPlace();
          }
          return centroids;
        } else {
          return std::nullopt;
        }
      });
}

auto Naive::GetWallCoordsLaserPlane(const LineSegment& left_surface, const LineSegment& right_surface,
                                    std::tuple<double, double, double>& left_surface_edges,
                                    std::tuple<double, double, double>& right_surface_edges, double left_groove_depth,
                                    double right_groove_depth) -> std::vector<std::tuple<double, double>> {
  auto [left_wall_angle, right_wall_angle] = GetMedianWallAngles();

  // left wall.
  auto x1 = std::get<0>(left_surface_edges);
  auto y1 = std::get<1>(left_surface_edges);
  auto k1 = -1. / tan(left_wall_angle);
  auto m1 = y1 - k1 * x1;
  LOG_TRACE("left_joint_angle: {}", left_wall_angle);

  // right wall
  auto x4 = std::get<0>(right_surface_edges);
  auto y4 = std::get<1>(right_surface_edges);
  auto k3 = 1. / tan(right_wall_angle);
  auto m3 = y4 - k3 * x4;
  LOG_TRACE("right_joint_angle: {}", right_wall_angle);

  auto k0 = left_surface.k;
  auto m0 = left_surface.m;
  LOG_TRACE("left top line: k0: {} m0: {}", k0, m0);

  // right top line
  auto k4 = right_surface.k;
  auto m4 = right_surface.m;
  LOG_TRACE("right top line: k0: {} m0: {}", k4, m4);

  // translate left top line "left_groove_depth" down
  // and calculate intersection with left wall
  // k1*x2+m1 = k0*x2 + m0 - left_groove_depth
  // -> (k1 - k0)*x2 = m0-m1 - left_groove_depth
  auto x2 = (m0 - m1 - left_groove_depth) / (k1 - k0);
  auto y2 = k1 * x2 + m1;

  // translate right top line "right_groove_depth" down
  // and calculate intersection with right wall
  // k3*x3+m3 = k4*x3 + m4 - right_groove_depth
  // -> (k3 - k4)*x3 = m4-m3 - right_groove_depth
  auto x3 = (m4 - m3 - right_groove_depth) / (k3 - k4);
  auto y3 = k3 * x3 + m3;

  auto xim = (x4 + x1) / 2.;
  auto yim = (y1 + y4) / 2.;

  std::vector<std::tuple<double, double>> wall_coords;
  wall_coords.push_back({x1, y1});
  wall_coords.push_back({x2, y2});
  wall_coords.push_back({x3, y3});
  wall_coords.push_back({x4, y4});
  wall_coords.push_back({xim, yim});

  return wall_coords;
}

auto Naive::TransformCoordsImagePlane(std::vector<std::tuple<double, double>>& wall_coords_laser, int vertical_offset)
    -> std::vector<std::tuple<int, int>> {
  auto [x1, y1]   = wall_coords_laser[0];
  auto [x2, y2]   = wall_coords_laser[1];
  auto [x3, y3]   = wall_coords_laser[2];
  auto [x4, y4]   = wall_coords_laser[3];
  auto [xim, yim] = wall_coords_laser[4];

  // Transform to image plane
  // -2.e-3 cuts out all data 2 mm under the toplines
  // y2 and y3 is lowered with 2mm compared wih python
  core::image::WorkspaceCoordinates mask_points_wcs(3, 5);
  mask_points_wcs << x1, x2, x3, x4, xim, y1 - 2.e-3, y2 + 3.e-3, y3 + 3.e-3, y4 - 2.e-3, yim, 0., 0., 0., 0., 0.;

  auto maybe_img = camera_model_->WorkspaceToImage(mask_points_wcs, vertical_offset);

  if (!maybe_img.has_value()) {
    LOG_ERROR("Not able to transform image to workspace");
    // return false;
  }

  auto img = maybe_img.value();

  // Mask
  auto c1 = int(round(img(0, 0)));
  auto r1 = int(round(img(1, 0)));

  auto c2 = int(round(img(0, 1)));
  auto r2 = int(round(img(1, 1)));

  auto c3 = int(round(img(0, 2)));
  auto r3 = int(round(img(1, 2)));

  auto c4 = int(round(img(0, 3)));
  auto r4 = int(round(img(1, 3)));

  auto cim = int(round(img(0, 4)));
  auto rim = int(round(img(1, 4)));

  LOG_TRACE("Mask: c1/r1: {}/{} c2/r2: {}/{} c3/r3: {}/{} c4/r4: {}/{} cim/rim: {}/{}", c1, r1, c2, r2, c3, r3, c4, r4,
            cim, rim);
  std::vector<std::tuple<int, int>> wall_coords_image;
  wall_coords_image.push_back({c1, r1});
  wall_coords_image.push_back({c2, r2});
  wall_coords_image.push_back({c3, r3});
  wall_coords_image.push_back({c4, r4});
  wall_coords_image.push_back({cim, rim});

  return wall_coords_image;
}

auto Naive::GetWallImage(std::vector<std::tuple<int, int>>& wall_coords_image, const core::image::RawImageData& image,
                         bool right) -> std::optional<std::tuple<const image::RawImageData, std::tuple<int, int>>> {
  if (right) {
    auto [c3, r3]     = wall_coords_image[2];
    auto [c4, r4]     = wall_coords_image[3];
    auto right_offset = std::max(c3 - 30, 0);
    auto right_length = c4 - right_offset;
    if (r3 <= r4 || right_length <= 0) {
      LOG_WARNING("Not able to crop image for right wall GetWallImages {} {} {}.", r4, r3, right_length);
      return {};
    }
    return {
        {image.block(r4, right_offset, r3 - r4, right_length), {right_offset, r4}}
    };
  } else {
    auto [c1, r1]    = wall_coords_image[0];
    auto [c2, r2]    = wall_coords_image[1];
    auto left_length = std::min(c2 + 30 - c1, static_cast<int>(image.cols()) - c1 - 1);
    if (r2 <= r1 || left_length <= 0) {
      LOG_WARNING("Not able to crop image for left wall GetWallImages {} {} {}.", r1, r2, left_length);
      return {};
    }
    return {
        {image.block(r1, c1, r2 - r1, left_length), {c1, r1}}
    };
  }
}

auto Naive::GetMedianWallAngles() -> std::tuple<double, double> {
  auto left_median  = properties_.left_joint_angle;
  auto right_median = properties_.right_joint_angle;
  if (!groove_wall_angles_buffer_.empty()) {
    std::vector<double> left_temp{};
    std::vector<double> right_temp{};

    for (auto& angles : groove_wall_angles_buffer_) {
      left_temp.push_back(angles.left_wall);
      right_temp.push_back(angles.right_wall);
    }

    left_median  = core::math::value::FindMedian(left_temp.begin(), left_temp.end(), left_temp.size());
    right_median = core::math::value::FindMedian(right_temp.begin(), right_temp.end(), right_temp.size());
  }
  auto left_median_lpcs  = LPCSFromWeldObjectAngle(left_median);
  auto right_median_lpcs = LPCSFromWeldObjectAngle(right_median);
  return {left_median_lpcs, right_median_lpcs};
}

auto Naive::GetMedianGrooveDepths() -> std::tuple<double, double> {
  // Returns average groove depth in meters, decreased by 5 mm.
  double left_median        = 0.0;
  double right_median       = 0.0;
  auto initial_groove_depth = properties_.groove_depth * 1.e-3;

  if (groove_wall_height_buffer_.empty()) {
    left_median  = initial_groove_depth;
    right_median = initial_groove_depth;
  } else {
    std::vector<double> left_temp{};
    std::vector<double> right_temp{};

    for (auto& depth : groove_wall_height_buffer_) {
      left_temp.push_back(depth.left_wall);
      right_temp.push_back(depth.right_wall);
    }

    left_median  = core::math::value::FindMedian(left_temp.begin(), left_temp.end(), left_temp.size());
    right_median = core::math::value::FindMedian(right_temp.begin(), right_temp.end(), right_temp.size());
  }

  LOG_TRACE("Groove depth: {} {}", left_median, right_median);
  return {left_median, right_median};
}

auto Naive::SurfaceLines(const Eigen::RowVectorXd& x, const Eigen::RowVectorXd& y)
    -> std::tuple<std::optional<LineSegment>, std::optional<LineSegment>> {
  using Eigen::Index;
  using Eigen::last;
  // Left top surface
  std::optional<LineSegment> left_line_model;
  {
    const double estimated_limit = x(0) + 0.9 * (x(last) - x(0) - (properties_.upper_joint_width * 1.0e-3)) / 2.0;
    auto limit                   = previous_left_edge.value_or(estimated_limit);
    LOG_TRACE("Calculating left surface line, up to {}.", limit);
    auto surface_indices = std::vector<Index>();
    for (Index i = 0; i < x.cols(); i++) {
      if (x[i] < limit) {
        surface_indices.push_back(i);
      }
    }
    LOG_TRACE("Left line indices: {}", surface_indices.size());
    if (surface_indices.size() > 1) {
      LOG_TRACE("First index: {} = {}", surface_indices[0], x[surface_indices[0]]);
      LOG_TRACE("Last index: {} = {}", surface_indices[surface_indices.size() - 1],
                x[surface_indices[surface_indices.size() - 1]]);
    } else {
      LOG_ERROR("No left surface line found");
      return {std::nullopt, std::nullopt};
    }

    auto line = FitPoints(x(surface_indices), y(surface_indices), 0.0005);

    if (abs(line.theta) <= properties_.left_max_surface_angle + properties_.surface_angle_tolerance) {
      left_line_model = line;
    } else {
      left_line_model = std::nullopt;
      LOG_WARNING("Left surface line above max limit. ({} > {})", abs(line.theta), properties_.left_max_surface_angle);
    }
  }

  // Right top surface
  std::optional<LineSegment> right_line_model;
  {
    const double estimated_limit = x(last) - 0.9 * (x(last) - x(0) - (properties_.upper_joint_width * 1.0e-3)) / 2.0;
    auto limit                   = previous_right_edge.value_or(estimated_limit);
    LOG_TRACE("Calculating right surface line, from {}", limit);
    auto surface_indices = std::vector<Index>();
    for (Index i = 0; i < x.cols(); i++) {
      if (x[i] >= limit) {
        surface_indices.push_back(i);
      }
    }
    LOG_TRACE("Right line indices: {}", surface_indices.size());
    if (surface_indices.size() > 1) {
      LOG_TRACE("First index: {} = {}", surface_indices[0], x[surface_indices[0]]);
      LOG_TRACE("Last index: {} = {}", surface_indices[surface_indices.size() - 1],
                x[surface_indices[surface_indices.size() - 1]]);
    } else {
      LOG_ERROR("No right surface line found");
      return {std::nullopt, std::nullopt};
    }

    auto line = FitPoints(x(surface_indices), y(surface_indices), 0.0005);

    if (abs(line.theta) <= properties_.right_max_surface_angle) {
      right_line_model = line;
    } else {
      right_line_model = std::nullopt;
      LOG_WARNING("Right surface line above max limit. ({} > {})", abs(line.theta),
                  properties_.right_max_surface_angle);
    }
  }

  return {left_line_model, right_line_model};
}

auto Naive::GrooveLine(std::optional<core::image::WorkspaceCoordinates> maybe_centroids) const
    -> std::optional<LineSegment> {
  if (!maybe_centroids.has_value()) {
    return std::nullopt;
  }
  auto centroids = maybe_centroids.value();
  if (centroids.cols() < 2) {
    LOG_WARNING("Not enough ({}) valid indices in groove line.", centroids.cols());
    return std::nullopt;
  }

  auto joint_wall_model = FitPoints(centroids.row(0), centroids.row(1), 0.0003);
  LOG_TRACE("Groove line: k: {} m: {} x_min: {} y_min: {} x_max {} y_max {}", joint_wall_model.k, joint_wall_model.m,
            joint_wall_model.x_limits.min, joint_wall_model.y_limits.min, joint_wall_model.x_limits.max,
            joint_wall_model.y_limits.max);

  return {joint_wall_model};
}

auto Naive::FitGrooveLine(const LineSegment& left_top_surface, const LineSegment& right_top_surface,
                          const LineSegment& left_wall, const LineSegment& right_wall,
                          const std::tuple<double, double, double>& left_surface_edge,
                          const std::tuple<double, double, double>& right_surface_edge) -> std::optional<PwlWedge> {
  auto maybe_left_intersection  = left_top_surface.intersection(left_wall);
  auto maybe_right_intersection = right_top_surface.intersection(right_wall);

  if (!maybe_left_intersection.has_value() || !maybe_right_intersection.has_value()) {
    return std::nullopt;
  }

  auto x0       = left_top_surface.x_limits.min;
  auto y0       = left_top_surface.k * x0 + left_top_surface.m;
  auto [x1, y1] = maybe_left_intersection.value();
  auto [x3, y3] = maybe_right_intersection.value();
  auto x4       = right_top_surface.x_limits.max;
  auto y4       = right_top_surface.k * x4 + right_top_surface.m;

  // Angles from RANSAC of walls are now good enough. x2,y2 is the intersection of
  // the two wall lines
  // left_wall.k * x2 + left_wall.m = right_wall.k * x2 + right_wall.m
  auto x2 = (right_wall.m - left_wall.m) / (left_wall.k - right_wall.k);
  auto y2 = left_wall.k * x2 + left_wall.m;

  auto x_left  = left_wall.x_limits.max;
  auto x_right = right_wall.x_limits.min;

  auto left_wall_angle_corrected  = LPCSToWeldObjectAngle(M_PI * 0.5 + left_wall.theta);
  auto right_wall_angle_corrected = LPCSToWeldObjectAngle(M_PI * 0.5 - right_wall.theta);

  auto min_width = (properties_.upper_joint_width - properties_.upper_joint_width_tolerance) * 1.0e-3;
  auto max_width = (properties_.upper_joint_width + properties_.upper_joint_width_tolerance) * 1.0e-3;

  // Check error conditions
  if ((x3 - x1) < min_width || ((x3 - x1) > max_width)) {
    LOG_WARNING("Groove width out of tolerance. {} {}", x3, x1);
    return {};
  }

  if ((left_wall_angle_corrected < (properties_.left_joint_angle - properties_.groove_angle_tolerance)) ||
      (left_wall_angle_corrected > (properties_.left_joint_angle + properties_.groove_angle_tolerance))) {
    LOG_WARNING("Left wall angle out of tolerance. {}", left_wall_angle_corrected);
    return {};
  }

  if ((right_wall_angle_corrected < (properties_.right_joint_angle - properties_.groove_angle_tolerance)) ||
      (right_wall_angle_corrected > (properties_.right_joint_angle + properties_.groove_angle_tolerance))) {
    LOG_WARNING("Right wall angle out of tolerance. {}", right_wall_angle_corrected);
    return {};
  }

  groove_wall_angles_buffer_.push_back({left_wall_angle_corrected, right_wall_angle_corrected});

  PwlWedge wedge{y0 / SP,
                 x1 / SP,
                 y1 / SP,
                 x2 / SP,
                 y2 / SP,
                 x3 / SP,
                 y3 / SP,
                 y4 / SP,
                 std::min(x_left, x_right),
                 std::max(x_left, x_right)};

  return wedge;
}

auto Naive::ExcludeCentroidsOutsideWedge(const PwlWedge& pwl_wedge, const Eigen::RowVectorXd& x,
                                         const Eigen::RowVectorXd& y)
    -> std::tuple<std::vector<double>, std::vector<double>> {
  std::vector<double> x_coords, y_coords;

  auto sign = [](double p1x, double p1y, double p2x, double p2y, double p3x, double p3y) {
    return (p1x - p3x) * (p2y - p3y) - (p2x - p3x) * (p1y - p3y);
  };

  const auto v1x   = pwl_wedge.x1t * SP - 0.001;
  const auto v2x   = pwl_wedge.x2t * SP;
  const auto v3x   = pwl_wedge.x3t * SP + 0.001;
  const auto v1y   = pwl_wedge.y1t * SP;
  const auto v2y   = pwl_wedge.y2t * SP - 0.002;  // Look 2mm below intersect
  const auto v3y   = pwl_wedge.y3t * SP;
  const auto y_lim = std::min(v1y, v3y) - properties_.groove_depth * 1.0e-3 - 2e-3;

  for (Eigen::Index i = 0; i < x.cols(); i++) {
    auto d1 = sign(x[i], y[i], v1x, v1y, v2x, v2y);
    auto d2 = sign(x[i], y[i], v2x, v2y, v3x, v3y);
    auto d3 = sign(x[i], y[i], v3x, v3y, v1x, v1y);
    if (y[i] > y_lim && !(((d1 < 0) || (d2 < 0) || (d3 < 0)) && ((d1 > 0) || (d2 > 0) || (d3 > 0)))) {
      x_coords.push_back(x[i]);
      y_coords.push_back(y[i]);
    }
  }

  return {x_coords, y_coords};
}

auto Naive::GrooveBottomLine(const Eigen::RowVectorXd& x, const Eigen::RowVectorXd& y, const PwlWedge& pwl_wedge,
                             bool empty_groove) -> std::optional<LineSegment> {
  using Eigen::Index;

  std::optional<LineSegment> groove_bottom_model;
  {
    // 2024-09-19 CURRENTLY NOT USED NOR TESTED
    // Special handling when groove is empty, i.e. when doing root pass.
    if (empty_groove) {
      std::vector<double> x_coords_interp;
      double x_coord = pwl_wedge.x1t;
      while (x_coord < pwl_wedge.x3t) {
        x_coords_interp.push_back(x_coord);
        x_coord += 0.1e-3;
      }
      x_coords_interp.push_back(pwl_wedge.x3t);
      std::vector<std::tuple<double, double>> segments;
      segments.emplace_back(pwl_wedge.x1t, pwl_wedge.y1t);
      segments.emplace_back(pwl_wedge.x2t, pwl_wedge.y2t);
      segments.emplace_back(pwl_wedge.x3t, pwl_wedge.y3t);

      auto y_coords_interp = core::math::lin_interp::lin_interp_2d(x_coords_interp, segments);

      LineSegment bottom(y_coords_interp.size());

      bottom.inliers.x =
          Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(x_coords_interp.data(), x_coords_interp.size());
      bottom.inliers.y =
          Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(y_coords_interp.data(), y_coords_interp.size());

      groove_bottom_model = bottom;
    }
    // Normal calculations when the groove is not empty
    else {
      // ----------------- Remove coordinates outside the triangle wedge ---------------------------
      auto [x_coords, y_coords] = ExcludeCentroidsOutsideWedge(pwl_wedge, x, y);

      if (x_coords.empty() || y_coords.empty()) {
        LOG_TRACE("No groove bottom coordinates found.");
        return {};
      }

      // -------- Filter groove bottom to exclude points differing too much from the median ------------
      // TODO: this filter should probably be made aware of the x coordinate
      const auto window_size_median_filter = std::clamp(y_coords.size() / 3, 20ul, 40ul);
      const auto y_coords_filter =
          core::math::filter::Median(Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(y_coords.data(), y_coords.size()),
                                     window_size_median_filter);

      assert(y_coords.size() == y_coords_filter.size());
      if (y_coords.size() != y_coords_filter.size()) {
        LOG_ERROR("Filtered y coords size doesn't match y coords size: {}, {}", y_coords_filter.size(),
                  y_coords.size());
      }

      for (auto i : std::views::iota(0u, y_coords.size()) | std::views::reverse) {
        if (fabs(y_coords[i] - y_coords_filter[i]) > 2.0e-3) {
          y_coords.erase(y_coords.begin() + i);
          x_coords.erase(x_coords.begin() + i);
        }
      }

      if (x_coords.empty() || y_coords.empty()) {
        LOG_TRACE("No groove bottom coordinates remaining after filtering.");
        return {};
      }

      //-------------------------------------------------------------------------------------------

      // ----------- Resample groove bottom in 0.1 mm increments ----------------------------------
      auto x_coords_interp = core::math::lin_interp::linspace(
          x_coords[0], x_coords[x_coords.size() - 1],
          1ul + static_cast<unsigned long>(std::llround((x_coords[x_coords.size() - 1] - x_coords[0]) / 0.1e-3)));

      std::vector<std::tuple<double, double>> segments;
      for (auto i : std::views::iota(0uz, std::min(x_coords.size(), y_coords.size()))) {
        segments.emplace_back(x_coords[i], y_coords[i]);
      }

      if (segments.size() < 2) {
        LOG_WARNING("Too few segments for interpolate.");
        return {};
      }

      auto y_coords_interp = core::math::lin_interp::lin_interp_2d(x_coords_interp, segments);
      //-------------------------------------------------------------------------------------------

      // ------------ Filter groove bottom using average filter of calculated window size ---------
      const auto window_size_average_filter = std::clamp(
          static_cast<unsigned long>(10.0 * (x_coords_interp[x_coords_interp.size() - 1] - x_coords_interp[0]) / 3.0),
          20ul, 50ul);

      y_coords = core::math::filter::Uniform1dReflect(y_coords_interp, window_size_average_filter);

      y_coords.insert(y_coords.begin(), *y_coords.begin());
      y_coords.insert(y_coords.end(), *(y_coords.end() - 1));
      x_coords = x_coords_interp;
      x_coords.insert(x_coords.begin(), pwl_wedge.x1t * SP);
      x_coords.insert(x_coords.end(), pwl_wedge.x3t * SP);
      //-------------------------------------------------------------------------------------------

      // ------------ Resample groove bottom ------------------------------------------------------
      if (pwl_wedge.x1t >= pwl_wedge.x3t) {
        LOG_WARNING("Not possible to resample groove bottom.");
        return {};
      }

      x_coords_interp =
          core::math::lin_interp::linspace(pwl_wedge.x1t * SP, pwl_wedge.x3t * SP,
                                           1ul + std::llround((pwl_wedge.x3t * SP - pwl_wedge.x1t * SP) / 0.1e-3));
      segments.clear();
      for (auto i : std::views::iota(0uz, std::min(x_coords.size(), y_coords.size()))) {
        segments.emplace_back(x_coords[i], y_coords[i]);
      }

      if (segments.size() < 2) {
        LOG_WARNING("Too few segments for interpolate.");
        return {};
      }

      y_coords_interp = core::math::lin_interp::lin_interp_2d(x_coords_interp, segments);
      //-------------------------------------------------------------------------------------------

      LineSegment bottom(y_coords_interp.size());
      bottom.inliers.x =
          Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(x_coords_interp.data(), x_coords_interp.size());
      bottom.inliers.y =
          Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(y_coords_interp.data(), y_coords_interp.size());

      groove_bottom_model = bottom;
    }
  }
  return groove_bottom_model;
}

auto Naive::ExtractABWPoints(const PwlWedge& pwl_wedge, const LineSegment& groove_bottom) -> std::optional<ABWPoints> {
  Eigen::Vector2d ABW0, ABW1, ABW2, ABW3, ABW4, ABW5, ABW6;

  // Take left wall line, translate 3 mm to right
  // Take right wall line, translate 3 mm to right
  // Calculate intersection of these two lines
  // If there are inliers below this point, use the wall intersection for ABW points.
  // Otherwise use the groove profile.

  auto dx_left  = pwl_wedge.x1t - pwl_wedge.x2t;
  auto dx_right = pwl_wedge.x3t - pwl_wedge.x2t;
  auto dy_left  = pwl_wedge.y1t - pwl_wedge.y2t;
  auto dy_right = pwl_wedge.y3t - pwl_wedge.y2t;
  if (dx_left == 0.0 || dx_right == 0.0) {
    LOG_WARNING("Vertical wall detected in ExtractABWPoints.");
    return std::nullopt;
  }

  auto k_left  = dy_left / dx_left;
  auto k_right = dy_right / dx_right;
  if (k_left == k_right) {
    LOG_WARNING("Parallel wall lines detected in ExtractABWPoints.");
    return std::nullopt;
  }

  auto CalcEdgeABWPoint = [](const Eigen::Vector2d& vp0, const Eigen::Vector2d& vp1, const LineSegment& bottom,
                             double offset) -> std::optional<Eigen::Vector2d> {
    // Construct the line from vp0 (bottom of wedge) in the direction of vp1.
    // The line is offset "offset" horizontally.
    auto dxi = vp1(0) - vp0(0);
    auto dyi = vp1(1) - vp0(1);
    if (std::abs(dxi) < 0.01e-3) {
      dxi = (dxi < 0) ? (-0.01e-3) : 0.01e-3;
    }
    auto ki = dyi / dxi;
    auto mi = vp0(1) - ki * (vp0(0) + offset);

    // Find the segment that intersects this line.
    for (unsigned long segment = 0; segment < bottom.inliers.x.size() - 1; segment++) {
      // Construct line for this segment
      auto x0 = bottom.inliers.x[segment];
      auto y0 = bottom.inliers.y[segment];
      auto x1 = bottom.inliers.x[segment + 1];
      auto y1 = bottom.inliers.y[segment + 1];
      if (x0 == x1) {
        LOG_WARNING("Vertical segment found in groove bottom profile.");
        return std::nullopt;
      }
      auto k_segment = (y1 - y0) / (x1 - x0);
      auto m_segment = y0 - k_segment * x0;

      // Get intersection between segment line and target line
      // k_segment * x + m_segment = ki * x + mi
      // (k_segment - ki) * x = mi - m_segment
      // x = (mi - m_segment) / (k_segment - ki)
      if (k_segment == ki) {
        // This segment is parallel to the wall. Strange, but we'll allow it.
        continue;
      }
      auto x = (mi - m_segment) / (k_segment - ki);

      // Check if the intersection is between the x limits of the segment.
      if (x >= x0 && x <= x1) {
        // Return the intersection point, offset by "offset" back
        return Eigen::Vector2d(x - offset, k_segment * x + m_segment);
      }
    }

    // We iterated over all bottom segments but did not find any intersection to the line.
    return std::nullopt;
  };

  auto v0p = Eigen::Vector2d(pwl_wedge.x2t * SP, pwl_wedge.y2t * SP);
  auto v1p = Eigen::Vector2d(pwl_wedge.x1t * SP, pwl_wedge.y1t * SP);
  auto v5p = Eigen::Vector2d(pwl_wedge.x3t * SP, pwl_wedge.y3t * SP);

  auto maybe_point = CalcEdgeABWPoint(v0p, v1p, groove_bottom, properties_.offset_distance * 1.0e-3);
  if (maybe_point.has_value()) {
    ABW1 = maybe_point.value();
  } else {
    LOG_WARNING("ABW1 does not have a value.");
    return std::nullopt;
  }

  maybe_point = CalcEdgeABWPoint(v0p, v5p, groove_bottom, -properties_.offset_distance * 1.0e-3);
  if (maybe_point.has_value()) {
    ABW5 = maybe_point.value();
  }

  auto dxi = (ABW5(0) - ABW1(0)) / 4.0;
  std::vector<std::tuple<double, double>> segments;
  auto y_index = groove_bottom.inliers.y.begin();
  for (auto x_index = groove_bottom.inliers.x.begin(); x_index < groove_bottom.inliers.x.end(); x_index++, y_index++) {
    segments.emplace_back(*x_index, *y_index);
  }
  auto ABW2_x = ABW1(0) + dxi;
  auto ABW2_y = core::math::lin_interp::lin_interp_2d(std::vector<double>{ABW2_x}, segments)[0];
  ABW2        = Eigen::Vector2d(ABW2_x, ABW2_y);

  auto ABW3_x = ABW1(0) + 2.0 * dxi;
  auto ABW3_y = core::math::lin_interp::lin_interp_2d(std::vector<double>{ABW3_x}, segments)[0];
  ABW3        = Eigen::Vector2d(ABW3_x, ABW3_y);

  auto ABW4_x = ABW1(0) + 3.0 * dxi;
  auto ABW4_y = core::math::lin_interp::lin_interp_2d(std::vector<double>{ABW4_x}, segments)[0];
  ABW4        = Eigen::Vector2d(ABW4_x, ABW4_y);

  ABW0 = Eigen::Vector2d(pwl_wedge.x1t * SP, pwl_wedge.y1t * SP);
  ABW6 = Eigen::Vector2d(pwl_wedge.x3t * SP, pwl_wedge.y3t * SP);

  LOG_INFO(
      "ABW_0: ({:.5f}, {:.5f}), ABW_1: ({:.5f}, {:.5f}), ABW_2: ({:.5f}, {:.5f}), ABW_3: ({:.5f}, {:.5f}), ABW_4: "
      "({:.5f}, {:.5f}), ABW_5: ({:.5f}, {:.5f}), ABW_6: "
      "({:.5f}, {:.5f})",
      ABW0[0], ABW0[1], ABW1[0], ABW1[1], ABW2[0], ABW2[1], ABW3[0], ABW3[1], ABW4[0], ABW4[1], ABW5[0], ABW5[1],
      ABW6[0], ABW6[1]);
  ABWPoints points = {
      Point{ABW0[0], ABW0[1]},
      Point{ABW1[0], ABW1[1]},
      Point{ABW2[0], ABW2[1]},
      Point{ABW3[0], ABW3[1]},
      Point{ABW4[0], ABW4[1]},
      Point{ABW5[0], ABW5[1]},
      Point{ABW6[0], ABW6[1]}
  };

  return points;
}

auto Naive::ExtractABWPointsCapWeldingMode(const std::optional<LineSegment> maybe_left_wall,
                                           const std::optional<LineSegment> maybe_right_wall,
                                           const LineSegment& left_surface, const LineSegment& right_surface,
                                           const core::image::WorkspaceCoordinates& joint) -> std::optional<ABWPoints> {
  auto wedge = wedge_buffer_.back();

  std::optional<PwlWedge> maybe_new_wedge =
      maybe_left_wall.and_then([left_surface](const LineSegment wall) { return wall.intersection(left_surface); })
          .transform([wedge](std::tuple<double, double> intersection) {
            auto [x_intersect, y_intersect] = intersection;
            return wedge.offset(x_intersect / SP - wedge.x1t, y_intersect / SP - wedge.y1t, SP);
          })
          .or_else([maybe_right_wall, right_surface, wedge]() {
            return maybe_right_wall
                .and_then([right_surface](const LineSegment wall) { return wall.intersection(right_surface); })
                .transform([wedge](std::tuple<double, double> intersection) {
                  auto [x_intersect, y_intersect] = intersection;
                  return wedge.offset(x_intersect / SP - wedge.x3t, y_intersect / SP - wedge.y3t, SP);
                });
          });

  if (maybe_new_wedge.has_value()) {
    wedge = maybe_new_wedge.value();
  }

  if (!maybe_new_wedge.has_value()) {
    // We have been unable to find any wall in the current frame.
    // The only thing we can do is move around the wedge

    // Check the difference between left surface y and left corner y and conversely
    // for the right side. Move the whole wedge up or down the average of these two
    // differences
    auto left_vertical_movement  = (left_surface.k * left_surface.x_limits.max + left_surface.m) / SP - wedge.y1t;
    auto right_vertical_movement = (right_surface.k * right_surface.x_limits.min + right_surface.m) / SP - wedge.y3t;
    wedge                        = wedge.offset(0, (left_vertical_movement + right_vertical_movement) / 2, SP);

    // Determine joint width based on this offset wedge
    auto x0 = wedge.left_wall_segment(SP)
                  .and_then([left_surface](LineSegment left_wall) -> std::optional<std::tuple<double, double>> {
                    return left_wall.intersection(left_surface);
                  })
                  .transform([](std::tuple<double, double> p) { return std::get<0>(p); })
                  .value_or(wedge.x1t * SP);
    auto x6 = wedge.right_wall_segment(SP)
                  .and_then([right_surface](LineSegment right_wall) -> std::optional<std::tuple<double, double>> {
                    return right_wall.intersection(right_surface);
                  })
                  .transform([](std::tuple<double, double> p) { return std::get<0>(p); })
                  .value_or(wedge.x3t * SP);

    // Calculate a vector of the difference between the ideal left surface and the measured
    // joint
    auto left_difference =
        joint.row(1) - (joint.row(0) * left_surface.k + Eigen::RowVectorXd::Ones(joint.rows()) * left_surface.m);
    auto right_difference =
        joint.row(1) - (joint.row(0) * right_surface.k + Eigen::RowVectorXd::Ones(joint.rows()) * right_surface.m);

    auto squared_left_difference  = left_difference.cwiseAbs2();
    auto squared_right_difference = right_difference.cwiseAbs2();

    double min_error     = std::numeric_limits<double>::max();
    double offset_at_min = 0.0;
    // Translate the wedge +- 0.3 mm in 0.1 mm increments
    for (double offset = -3e-4; offset < 3e-4; offset += 3e-5) {
      // Sum left difference up to x0 + offset, then right difference from x6 + offset to end
      double sum  = 0;
      int n_items = 0;
      for (int x_index = 0; x_index < joint.cols(); x_index++) {
        if (joint(0, x_index) < x0 + offset) {
          sum += fabs(squared_left_difference(x_index));
          n_items++;
        } else if (joint(0, x_index) > x6 + offset) {
          sum += fabs(squared_right_difference(x_index));
          n_items++;
        }
      }
      auto average = (n_items > 0) ? sum / n_items : std::numeric_limits<double>::max();

      if (average < min_error) {
        min_error     = sum;
        offset_at_min = offset;
      }
    }

    wedge = wedge.offset(offset_at_min * SP, 0, SP);
  }

  wedge_buffer_.push_back(wedge);
  auto filtered_joint = math::filter::Median(joint.row(1), 5);

  // Intersect wedge with surface lines again.
  auto [x0, y0] = wedge.left_wall_segment(SP)
                      .and_then([left_surface](LineSegment left_wall) -> std::optional<std::tuple<double, double>> {
                        return left_wall.intersection(left_surface);
                      })
                      .value_or(std::make_tuple(wedge.x1t * SP, wedge.y1t * SP));
  auto [x6, y6] = wedge.right_wall_segment(SP)
                      .and_then([right_surface](LineSegment right_wall) -> std::optional<std::tuple<double, double>> {
                        return right_wall.intersection(right_surface);
                      })
                      .value_or(std::make_tuple(wedge.x3t * SP, wedge.y3t * SP));

  auto y_at_x = [joint, filtered_joint](double x) -> double {
    int col;
    for (col = 0; col < joint.cols() && joint(0, col) < x; col++);
    if (col == joint.cols()) {
      return filtered_joint(col - 1);
    } else {
      return filtered_joint(col);
    }
  };
  auto x3 = (x0 + x6) / 2;
  auto y3 = y_at_x(x3);
  auto x1 = x0 + (y0 - y3) * tan(properties_.left_joint_angle);
  auto y1 = y_at_x(x1 + properties_.offset_distance * 1.0e-3);
  auto x2 = (x1 + x3) / 2;
  auto y2 = y_at_x(x2);
  auto x5 = x6 - (y6 - y3) * tan(properties_.right_joint_angle);
  auto y5 = y_at_x(x5 - properties_.offset_distance * 1.0e-3);
  auto x4 = (x3 + x5) / 2;
  auto y4 = y_at_x(x4);

  Point abw0 = Point{x0, y0};
  Point abw1 = Point{x1, y1};
  Point abw2 = Point{x2, y2};
  Point abw3 = Point{x3, y3};
  Point abw4 = Point{x4, y4};
  Point abw5 = Point{x5, y5};
  Point abw6 = Point{x6, y6};

  LOG_INFO(
      "CAP WELDING ABW_0: ({:.5f}, {:.5f}), ABW_1: ({:.5f}, {:.5f}), ABW_2: ({:.5f}, {:.5f}), ABW_3: ({:.5f}, {:.5f}), "
      "ABW_4: ({:.5f}, {:.5f}), ABW_5: ({:.5f}, {:.5f}), ABW_6: "
      "({:.5f}, {:.5f})",
      x0, y0, x1, y1, x2, y2, x3, y3, x4, y4, x5, y5, x6, y6);

  ABWPoints points = {abw0, abw1, abw2, abw3, abw4, abw5, abw6};

  return points;
}

auto Naive::GetImprovedSurfaceLine(const core::image::WorkspaceCoordinates& laser_line, const LineSegment& line_segment,
                                   bool reverse, double tolerance) -> std::optional<LineSegment> {
  using namespace boost::accumulators;
  using Eigen::Index;
  using Eigen::last;
  using Eigen::RowVectorXd;

  PlaneCoordinates line(2, laser_line.cols());
  line << laser_line.row(0), laser_line.row(1);

  // Coarse search
  PlaneCoordinates calculated = line;
  calculated.row(1)           = line_segment.k * calculated.row(0).array() + line_segment.m;

  if (reverse) {
    // The right side top line
    calculated.row(1).reverseInPlace();
    line.row(0).reverseInPlace();
    line.row(1).reverseInPlace();
  }

  // Filter out the groove
  auto indices = std::vector<Index>();

  // When we have found more than 20 indices, if there are N consecutive indices out of tolerance, stop
  int consecutive_indices_below = 0;
  for (Index i = 0; i < calculated.cols(); i++) {
    const bool above_calculated   = line(1, i) - calculated(1, i) > 2.0e-3;
    const bool below_calculated   = calculated(1, i) - line(1, i) > 4.0e-3 && calculated(1, i) - line(1, i) < 10.0e-3;
    const bool found_more_than_20 = indices.size() > 20;

    if (found_more_than_20 && below_calculated) {
      if (++consecutive_indices_below >= 5) {
        break;
      }
    }
    if (!above_calculated && !below_calculated) {
      consecutive_indices_below = 0;
      indices.push_back(i);
    }
  }

  if (indices.size() < 2) {
    LOG_WARNING("Not enough ({}) valid indices in search end point.", indices.size());
    return std::nullopt;
  }

  // Do a new fit points. The end point of the inliers of this line is the new end point.
  return FitPoints(line(0, indices), line(1, indices), tolerance);
}

#ifndef DOCTEST_CONFIG_DISABLE
// NOLINTBEGIN(*-magic-numbers)

#include <doctest/doctest.h>

#include "test_data.h"

using Eigen::Index;
using Eigen::Matrix3d;
using Eigen::RowVectorX;
using Eigen::RowVectorXd;
using Eigen::Vector3d;

using core::image::CameraProperties;
using core::image::RawImageData;
using core::image::WorkspaceCoordinates;
using core::scanner::JointProperties;

/**
 * This function is used to bootstrap tests. It generates a point cloud
 * vaguely resembling a laser line.
 * @param image_width
 * @param image_height
 * @param width
 * @param height
 * @param surface_height
 * @param left_surface_angle
 * @param right_surface_angle
 * @param groove_width
 * @param groove_angle
 * @return
 */
static auto SetupTest(int image_width, int image_height, double width, double height, double surface_height,
                      double left_surface_angle, double right_surface_angle, double groove_width, double groove_angle)
    -> std::tuple<RowVectorXd, RowVectorXd, RowVectorXd, RowVectorXd, RowVectorXd, RowVectorXd> {
  const double left_surface_angle_radians  = left_surface_angle;
  const double right_surface_angle_radians = right_surface_angle;
  const double groove_angle_radians        = groove_angle;

  RowVectorXd x = RowVectorXd::LinSpaced(image_width, 0.0, width);
  RowVectorXd y(image_width);

  LOG_TRACE("Start: {}, {}", 0.0, surface_height);

  // Left surface
  double left_corner_x = NAN;
  double left_corner_y = NAN;
  Index left_corner_index;
  for (Index i = 0; i < image_width; i++) {
    if (x[i] <= (width - groove_width) / 2.0) {
      y[i]              = surface_height + tan(left_surface_angle_radians) * x[i];
      left_corner_x     = x[i];
      left_corner_y     = y[i];
      left_corner_index = i;
    }
  }

  LOG_TRACE("Left corner: {}, {}", left_corner_x, left_corner_y);

  // Left groove line
  double groove_bottom_x = NAN;
  double groove_bottom_y = NAN;
  for (Index i = 0; i < image_width; i++) {
    if (x[i] >= left_corner_x && x[i] <= left_corner_x + groove_width / 2.0) {
      y[i]            = left_corner_y - tan(groove_angle_radians / 2.0) * (x[i] - left_corner_x);
      groove_bottom_x = x[i];
      groove_bottom_y = y[i];
    }
  }

  LOG_TRACE("Mid: {}, {}", groove_bottom_x, groove_bottom_y);

  // Right groove line
  double groove_right_x = NAN;
  double groove_right_y = NAN;
  Index right_corner_index;
  for (Index i = 0; i < image_width; i++) {
    if (x[i] >= groove_bottom_x && x[i] <= left_corner_x + groove_width) {
      y[i]               = groove_bottom_y + tan(groove_angle_radians / 2.0) * (x[i] - groove_bottom_x);
      groove_right_x     = x[i];
      groove_right_y     = y[i];
      right_corner_index = i;
    }
  }

  LOG_TRACE("Right corner: {}, {}", groove_right_x, groove_right_y);

  // Right surface
  double end_x = NAN;
  double end_y = NAN;
  for (Index i = 0; i < image_width; i++) {
    if (x[i] >= groove_right_x) {
      y[i]  = groove_right_y + tan(right_surface_angle_radians) * (x[i] - groove_right_x);
      end_x = x[i];
      end_y = y[i];
    }
  }

  LOG_TRACE("End: {}, {}", end_x, end_y);

  // Extract groove
  RowVectorXd left_wall_x  = x.block(0, left_corner_index, 1, image_width / 2.0 - left_corner_index);
  RowVectorXd left_wall_y  = y.block(0, left_corner_index, 1, image_width / 2.0 - left_corner_index);
  RowVectorXd right_wall_x = x.block(0, x.cols() / 2, 1, right_corner_index - x.cols() / 2);
  RowVectorXd right_wall_y = y.block(0, y.cols() / 2, 1, right_corner_index - y.cols() / 2);

  return {x, y, left_wall_x, left_wall_y, right_wall_x, right_wall_y};
}

class NaiveExposed : public Naive {
 public:
  explicit NaiveExposed(const JointProperties& properties,
                        const core::scanner::ScannerConfigurationData& scanner_config)
      : Naive(properties, scanner_config, core::image::CameraModelPtr{}) {};
  using core::scanner::joint_model::Naive::FitGrooveLine;
  using core::scanner::joint_model::Naive::GetImprovedSurfaceLine;
  using core::scanner::joint_model::Naive::GetMedianGrooveDepths;
  using core::scanner::joint_model::Naive::groove_wall_height_buffer_;
  using core::scanner::joint_model::Naive::GrooveBottomLine;
  using core::scanner::joint_model::Naive::GrooveLine;
  using core::scanner::joint_model::Naive::SurfaceLines;
};

TEST_SUITE("Naive Joint Model") {
  TEST_CASE("Surface line find") {
    const int image_width            = 3500;
    const int image_height           = 2000;
    const double width               = 120.0 * 1.0e-3;
    const double height              = image_height / static_cast<double>(image_width) * width;
    const double surface_height      = 30.0 * 1.0e-3;
    const double left_surface_angle  = 10.0 * static_cast<double>(EIGEN_PI) / 180.0;
    const double right_surface_angle = 4.0 * static_cast<double>(EIGEN_PI) / 180.0;
    const double groove_width        = 40.0 * 1.0e-3;
    const double groove_angle        = 60.0 * static_cast<double>(EIGEN_PI) / 180.0;

    auto joints = SetupTest(image_width, image_height, width, height, surface_height, left_surface_angle,
                            right_surface_angle, groove_width, groove_angle);

    auto x = std::get<0>(joints);
    auto y = std::get<1>(joints);

    JointProperties joint_properties = {};

    joint_properties.surface_angle_tolerance = 10.0 * static_cast<double>(EIGEN_PI) / 180.0;
    joint_properties.left_max_surface_angle  = left_surface_angle + 0.1 * static_cast<double>(EIGEN_PI) / 180.0;
    joint_properties.right_max_surface_angle = right_surface_angle + 0.1 * static_cast<double>(EIGEN_PI) / 180.0;
    joint_properties.upper_joint_width       = groove_width * 1.0e3;

    NaiveExposed exposed(joint_properties, {16, 16, 16});
    auto [left, right] = exposed.SurfaceLines(x, y);

    if (left.has_value()) {
      LOG_TRACE("{}", left.value().ToString());
      CHECK_LE(abs(left.value().theta - left_surface_angle), 0.001);
    } else {
      CHECK(false);
    }

    if (right.has_value()) {
      LOG_TRACE("{}", right.value().ToString());
      CHECK_LE(abs(right.value().theta - right_surface_angle), 0.001);
    } else {
      CHECK(false);
    }
  }

  TEST_CASE("Groove line find") {
    using Eigen::Matrix;

    const int image_width            = 3500;
    const int image_height           = 2000;
    const double width               = 120.0 * 1.0e-3;
    const double height              = image_height / static_cast<double>(image_width) * width;
    const double surface_height      = 30.0 * 1.0e-3;
    const double left_surface_angle  = 10.0 * static_cast<double>(EIGEN_PI) / 180.0;
    const double right_surface_angle = 4.0 * static_cast<double>(EIGEN_PI) / 180.0;
    const double groove_width        = 40.0 * 1.0e-3;
    const double groove_angle        = 60.0 * static_cast<double>(EIGEN_PI) / 180.0;

    auto joints = SetupTest(image_width, image_height, width, height, surface_height, left_surface_angle,
                            right_surface_angle, groove_width, groove_angle);

    auto left_wall_x  = std::get<2>(joints);
    auto left_wall_y  = std::get<3>(joints);
    auto right_wall_x = std::get<4>(joints);
    auto right_wall_y = std::get<5>(joints);

    JointProperties joint_properties = {};

    joint_properties.upper_joint_width           = groove_width * 1.0e3;
    joint_properties.upper_joint_width_tolerance = 1.0;

    Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor> left_centroids(3, left_wall_x.cols());
    left_centroids.block(0, 0, 1, left_wall_x.cols()) = left_wall_x;
    left_centroids.block(1, 0, 1, left_wall_x.cols()) = left_wall_y;

    NaiveExposed exposed(joint_properties, {16, 16, 16});
    auto maybe_left_groove_line = exposed.GrooveLine(left_centroids);

    CHECK(maybe_left_groove_line.has_value());

    CHECK_LE(abs(maybe_left_groove_line.value().theta + groove_angle / 2.0), 0.001);
  }

  TEST_CASE("Fit wedge") {
    JointProperties joint_properties = {};

    joint_properties.upper_joint_width           = 50.0 * 1e3;
    joint_properties.left_joint_angle            = 30.0 * static_cast<double>(EIGEN_PI) / 180.0;
    joint_properties.right_joint_angle           = 30.0 * static_cast<double>(EIGEN_PI) / 180.0;
    joint_properties.upper_joint_width_tolerance = 5.0 * 1e3;
    joint_properties.groove_angle_tolerance      = 5.0 * static_cast<double>(EIGEN_PI) / 180.0;

    NaiveExposed exposed(joint_properties, {16, 16, 16});

    LineSegment left_top_surface(100);
    left_top_surface.k = 2.4188305251300335e-03;
    left_top_surface.m = 1.0309065133333206e-01;
    LineSegment right_top_surface(100);
    right_top_surface.k = -1.5275700949132442e-02;
    right_top_surface.m = 1.0255440324544907e-01;

    std::tuple<double, double, double> left_surface_edge  = {-3.9591481451023654e-02, 1.0299610130067234e-01, 0.0};
    std::tuple<double, double, double> right_surface_edge = {5.7430421588993263e-03, 1.0247409568516797e-01, 0.0};

    LineSegment left_wall(left_wall_x.size());
    LineSegment right_wall(right_wall_x.size());

    left_wall.inliers.x  = Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(left_wall_x.data(), left_wall_x.size());
    left_wall.inliers.y  = Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(left_wall_y.data(), left_wall_y.size());
    right_wall.inliers.x = Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(right_wall_x.data(), right_wall_x.size());
    right_wall.inliers.y = Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(right_wall_y.data(), right_wall_y.size());

    left_wall.x_limits.min  = left_wall_x[0];
    left_wall.x_limits.max  = left_wall_x[left_wall_x.size() - 1];
    right_wall.x_limits.min = right_wall_x[0];
    right_wall.x_limits.max = right_wall_x[right_wall_x.size() - 1];
    left_wall.y_limits.min  = left_wall_y[0];
    left_wall.y_limits.max  = left_wall_y[left_wall_y.size() - 1];
    right_wall.y_limits.min = right_wall_y[0];
    right_wall.y_limits.max = right_wall_y[right_wall_y.size() - 1];
    left_wall.theta         = -1.05;
    right_wall.theta        = 1.05;

    auto maybe_pwl_wedge = exposed.FitGrooveLine(left_top_surface, right_top_surface, left_wall, right_wall,
                                                 left_surface_edge, right_surface_edge);
    if (maybe_pwl_wedge.has_value()) {
      auto pwl_wedge = maybe_pwl_wedge.value();
      CHECK_LE(abs(pwl_wedge.y0t - 10.3091), 0.0001);
      // TODO: Estimate estimate k+m values for left_wall and right_wall so that x1,y1,x2,y2,x3,y3 will be defined.
      // CHECK_LE(abs(pwl_wedge.x1t - (-3.96195963)), 0.0001);
      // CHECK_LE(abs(pwl_wedge.y1t - 10.2995), 0.0001);
      // CHECK_LE(abs(pwl_wedge.x2t - (-1.91182607)), 0.0001);
      // CHECK_LE(abs(pwl_wedge.y2t - 6.58339778), 0.0001);
      // CHECK_LE(abs(pwl_wedge.x3t - 0.58522478), 0.0001);
      // CHECK_LE(abs(pwl_wedge.y3t - 10.24650061), 0.0001);
      CHECK_LE(abs(pwl_wedge.y4t - 10.2554), 0.0001);
    } else {
      CHECK(false);
    }
  }

  TEST_CASE("Groove bottom") {
    JointProperties joint_properties = {};

    joint_properties.upper_joint_width           = 45.0;
    joint_properties.left_joint_angle            = 30.0 * static_cast<double>(EIGEN_PI) / 180.0;
    joint_properties.right_joint_angle           = 30.0 * static_cast<double>(EIGEN_PI) / 180.0;
    joint_properties.upper_joint_width_tolerance = 5.0;
    joint_properties.groove_angle_tolerance      = 5.0 * static_cast<double>(EIGEN_PI) / 180.0;
    joint_properties.groove_depth                = 60.0;

    NaiveExposed exposed(joint_properties, {16, 16, 16});

    Eigen::RowVectorXd x_coords(lpcs_x_coords.size());
    Eigen::RowVectorXd z_coords(lpcs_z_coords.size());

    x_coords = Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(lpcs_x_coords.data(), lpcs_x_coords.size());
    z_coords = Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(lpcs_z_coords.data(), lpcs_z_coords.size());

    LineSegment left_wall(left_wall_x.size());
    LineSegment right_wall(right_wall_x.size());

    left_wall.inliers.x  = Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(left_wall_x.data(), left_wall_x.size());
    left_wall.inliers.y  = Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(left_wall_y.data(), left_wall_y.size());
    right_wall.inliers.x = Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(right_wall_x.data(), right_wall_x.size());
    right_wall.inliers.y = Eigen::Map<Eigen::RowVectorXd, Eigen::Unaligned>(right_wall_y.data(), right_wall_y.size());

    left_wall.x_limits.max  = -0.021572690357988252;
    right_wall.x_limits.min = -0.01575632277317308;
    left_wall.y_limits.min  = 7.0282891392707825e-02;
    right_wall.y_limits.min = 7.0282891392707825e-02;

    core::scanner::PwlWedge pwl_wedge{
        .y0t     = 1.0299733308498569e+01,
        .x1t     = -3.9619596192279607e+00,
        .y1t     = 1.0299481824466884e+01,
        .x2t     = -1.9118260710987687e+00,
        .y2t     = 6.5833977763209042e+00,
        .x3t     = 5.8522477515062754e-01,
        .y3t     = 1.0246500605891683e+01,
        .y4t     = 1.0245704293258299e+01,
        .x_left  = -2.1572690418818225e-02,
        .x_right = -1.5756322752176505e-02,
    };

    exposed.GrooveBottomLine(x_coords, z_coords, pwl_wedge);
  }
}

// NOLINTEND(*-magic-numbers)
#endif
