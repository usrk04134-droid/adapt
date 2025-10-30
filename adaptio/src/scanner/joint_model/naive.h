#pragma once

#include <boost/circular_buffer.hpp>
#include <Eigen/Eigen>
#include <expected>
#include <tuple>

#include "scanner/image/camera_model.h"
#include "scanner/image/image_types.h"
#include "scanner/joint_model/joint_model.h"
#include "scanner/scanner_configuration.h"

namespace scanner::joint_model {

struct PwlWedge {
  double y0t;
  double x1t;
  double y1t;
  double x2t;
  double y2t;
  double x3t;
  double y3t;
  double y4t;
  double x_left;   // Not scaled
  double x_right;  // Not scaled

  friend auto operator<<(std::ostream& output_stream, const PwlWedge& pwl) -> std::ostream& {
    output_stream << std::endl
                  << "\ty0t: " << pwl.y0t << std::endl
                  << "\ty1t: " << pwl.y1t << "\tx1t: " << pwl.x1t << std::endl
                  << "\ty2t: " << pwl.y2t << "\tx2t: " << pwl.x2t << std::endl
                  << "\ty3t: " << pwl.y3t << "\tx3t: " << pwl.x3t << std::endl
                  << "\ty4t: " << pwl.y4t << std::endl
                  << "\tx_left: " << pwl.x_left << "\tx_right: " << pwl.x_right << std::endl;
    return output_stream;
  }

  auto offset(double delta_x, double delta_y, double SP) const -> PwlWedge {
    return {y0t + delta_y, x1t + delta_x, y1t + delta_y, x2t + delta_x,         y2t + delta_y,
            x3t + delta_x, y3t + delta_y, y4t + delta_y, x_left + delta_x * SP, x_right + delta_y * SP};
  }

  auto left_wall_segment(double SP) const -> std::optional<LineSegment> {
    if (x1t == x2t) {
      return std::nullopt;
    } else {
      auto k = (y1t - y2t) / (x1t - x2t);
      auto m = y2t * SP - k * x2t * SP;
      return LineSegment(k, m);
    }
  }

  auto right_wall_segment(double SP) const -> std::optional<LineSegment> {
    if (x3t == x2t) {
      return std::nullopt;
    } else {
      auto k = (y3t - y2t) / (x3t - x2t);
      auto m = y2t * SP - k * x2t * SP;
      return LineSegment(k, m);
    }
  }
};

class Naive : public JointModel {
 public:
  Naive(const JointProperties& properties, const ScannerConfigurationData& config_data,
        image::CameraModelPtr camera_model);

  Naive(const Naive&)                        = delete;
  auto operator=(const Naive&) -> Naive&     = delete;
  Naive(Naive&&) noexcept                    = delete;
  auto operator=(Naive&&) noexcept -> Naive& = delete;

  ~Naive() = default;

  /**
   * Tries to calculate ABW points from the input image coordinates.
   *
   * @param joint   Laser line coordinates
   * @return The identified groove information from the laser line coordinates.
   */
  auto Parse(image::Image& image, std::optional<JointProfile> median_profile,
             std::optional<JointProperties> empty_joint_properties, bool use_approximation,
             std::optional<std::tuple<double, double>> abw0_abw6_horizontal)
      -> std::expected<std::tuple<JointProfile, image::WorkspaceCoordinates, uint64_t, uint64_t>,
                       JointModelErrorCode> override;

 protected:
  /**
   * Calculates the area of the joint given the bottom centroids and the top surface edges.
   *
   * @param bottom_centroids    Bottom centroids
   * @param pwl_wedge           The wedge
   * @return The area in m^2 in laser plane coordinates.
   */
  [[nodiscard]] auto GetJointArea(image::WorkspaceCoordinates centroids, const PwlWedge& pwl_wedge) -> double;

  /**
   * Helper function that returns filtered lists of bottom centroids to
   * include only those within the wedge.
   *
   * @param pwl_wedge           The wedge
   * @param x                   Centroid x values
   * @param y                   Centroid y values
   * @return The area in mm^2 in laser plane coordinates.
   */
  [[nodiscard]] auto ExcludeCentroidsOutsideWedge(const PwlWedge& pwl_wedge, const Eigen::RowVectorXd& x,
                                                  const Eigen::RowVectorXd& y)
      -> std::tuple<std::vector<double>, std::vector<double>>;

  /**
   * Tries to find the left and right top surface lines.
   *
   * @param x               X values of the laser line centroids
   * @param y               Y values of the laser line centroids
   * @return The left and right top surface lines if found.
   */
  [[nodiscard]] auto SurfaceLines(const Eigen::RowVectorXd& x, const Eigen::RowVectorXd& y)
      -> std::tuple<std::optional<LineSegment>, std::optional<LineSegment>>;

  /**
   * Tries to find a groove line based on row centroids.
   *
   * @param centroids   Optionally, x and y values of the row centroids.
   * @return The groove lines if found.
   */
  [[nodiscard]] auto GrooveLine(std::optional<image::WorkspaceCoordinates> centroids) const
      -> std::optional<LineSegment>;

  /**
   * Tries to fit a piece wise linear wedge to the identified left and right groove lines.
   *
   * @param left_top_surface    Left top surface line segment
   * @param right_top_surface   Right top surface line segment
   * @param left_wall           Left groove line segment
   * @param right_wall          Right groove line segment
   * @param left_surface_edge   Left surface right most endpoint
   * @param right_surface_edge  Right surface left most endpoint
   * @return The piece wise linear wedge if found.
   */
  [[nodiscard]] auto FitGrooveLine(const LineSegment& left_top_surface, const LineSegment& right_top_surface,
                                   const LineSegment& left_wall, const LineSegment& right_wall,
                                   const std::tuple<double, double, double>& left_surface_edge,
                                   const std::tuple<double, double, double>& right_surface_edge)
      -> std::optional<PwlWedge>;

  /**
   * Tries to find the groove bottom line.
   *
   * @param x               X values of the laser line centroids
   * @param y               Y values of the laser line centroids
   * @param left_wall       Left grove line segment
   * @param right_wall      Right grove line segment
   * @param pwl_wedge       The piece wise linear wedge fitting the groove
   * @param empty_groove    If groove is empty or if it has started to be filled.
   * @return The groove bottom line if found.
   */
  auto GrooveBottomLine(const Eigen::RowVectorXd& x, const Eigen::RowVectorXd& y, const PwlWedge& pwl_wedge,
                        bool empty_groove = false) -> std::optional<LineSegment>;

  /**
   * Extracts ABW points from the piece wise linear wedge and groove bottom line.
   *
   * @param pwl_wedge       The piece wise linear wedge fitting the groove
   * @param groove_bottom   The groove bottomline
   * @return ABW points of the analyzed image.
   */
  auto ExtractABWPoints(const PwlWedge& pwl_wedge, const LineSegment& groove_bottom) -> std::optional<ABWPoints>;

  /**
   * Extracts ABW points from the joint centroidsnear wedge and groove bottom line.
   *
   * @param pwl_wedge       The piece wise linear wedge fitting the groove
   * @param groove_bottom   The groove bottomline
   * @return ABW points of the analyzed image.
   */
  auto ExtractABWPointsCapWeldingMode(const std::optional<LineSegment> maybe_left_wall,
                                      const std::optional<LineSegment> maybe_right_wall,
                                      const LineSegment& left_surface, const LineSegment& right_surface,
                                      const image::WorkspaceCoordinates& joint) -> std::optional<ABWPoints>;

  /**
   * Tries to improve upon the previously found surface line by only repeating the line
   * fitting with a subset of laser line points (based on the previous line).
   *
   * @param laser_line
   * @param line_segment
   * @param reverse
   * @param tolerance
   * @return A new better line.
   */
  auto GetImprovedSurfaceLine(const image::WorkspaceCoordinates& laser_line, const LineSegment& line_segment,
                              bool reverse, double tolerance) -> std::optional<LineSegment>;

  /**
   * Tries to calculate centroids for left and right wall.
   *
   * @param surface          k, m for left top line is used to calculate the mask.
   * @param surface_edges    Left top corner is used to calculate the mask.
   * @param src_image             The actual image to process.
   * @return Centroids for the wall.
   */
  auto GetWallCentroids(const LineSegment& surface, const std::tuple<double, double, double>& surface_edges,
                        const image::RawImageData& image, std::tuple<int, int> offset, int extra_vertical_offset,
                        bool right_wall, int horizontal_crop_offset)
      -> std::optional<image::WorkspaceCoordinates>;

  auto GetMedianGrooveDepths() -> std::tuple<double, double>;

  struct groove_wall_heights {
    double left_wall;
    double right_wall;
  };
  boost::circular_buffer<groove_wall_heights> groove_wall_height_buffer_;

 private:
  auto GetBottomCentroids(const image::Image& image, const LineSegment& left_groove_wall,
                          const LineSegment& right_groove_wall, const PwlWedge& pwl_wedge)
      -> std::optional<image::WorkspaceCoordinates>;

  auto GetMedianWallAngles() -> std::tuple<double, double>;

  auto GetWallCoordsLaserPlane(const LineSegment& left_surface, const LineSegment& right_surface,
                               std::tuple<double, double, double>& left_surface_edges,
                               std::tuple<double, double, double>& right_surface_edges, double left_groove_depth,
                               double right_groove_depth) -> std::vector<std::tuple<double, double>>;

  auto TransformCoordsImagePlane(std::vector<std::tuple<double, double>>& wall_coords_laser, int vertical_offset)
      -> std::vector<std::tuple<int, int>>;

  auto GetWallImage(std::vector<std::tuple<int, int>>& wall_coords_image, const image::RawImageData& image, bool right)
      -> std::optional<std::tuple<const image::RawImageData, std::tuple<int, int>>>;

  struct groove_wall_angles {
    double left_wall;
    double right_wall;
  };
  boost::circular_buffer<groove_wall_angles> groove_wall_angles_buffer_;
  boost::circular_buffer<PwlWedge> wedge_buffer_;

  std::optional<double> previous_left_edge  = std::nullopt;
  std::optional<double> previous_right_edge = std::nullopt;
  uint8_t gray_minimum_top;
  uint8_t gray_minimum_wall;
  uint8_t gray_minimum_bottom;

  enum { WELD_MODE_REGULAR, WELD_MODE_CAP } welding_mode_ = WELD_MODE_REGULAR;

  int consecutive_frames_without_finding_both_walls = 0;
};

}  // namespace scanner::joint_model
