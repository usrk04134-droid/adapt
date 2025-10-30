#pragma once

#include <fmt/format.h>

#include <boost/outcome.hpp>
#include <Eigen/Eigen>
#include <expected>
#include <string>
#include <vector>

#include "scanner/core/scanner_configuration.h"
#include "scanner/image/camera_model.h"

namespace scanner::joint_model {

const double HIGH_CONFIDENCE_WALL_HEIGHT   = 0.007;
const double MEDIUM_CONFIDENCE_WALL_HEIGHT = 0.006;

struct Point {
  double x = 0;
  double y = 0;
  friend auto operator<<(std::ostream& output_stream, const Point& p) -> std::ostream& {
    output_stream << "(" << p.x << "," << p.y << ")";
    return output_stream;
  }
  auto operator+(const Point& o) -> Point { return Point{x + o.x, y + o.y}; }
  auto operator-(const Point& o) -> Point { return Point{x - o.x, y - o.y}; }
  auto operator*(double f) -> Point { return Point{x * f, y * f}; }

  auto TranslatedHorizontally(double offset) -> Point { return Point{x + offset, y}; }
};

using ABWPoints = std::array<Point, 7>;

inline auto ABWPointsToMatrix(ABWPoints a) -> image::WorkspaceCoordinates {
  auto out = image::WorkspaceCoordinates(3, 7);
  out.setZero();
  for (int i = 0; i < 7; i++) {
    out(0, i) = a[i].x;
    out(1, i) = a[i].y;
  }
  return out;
}

enum class JointModelErrorCode : uint32_t {
  NO_ERROR                         = 0,
  SURFACE_NOT_FOUND                = 1,
  WEDGE_FIT_FAILED                 = 3,
  GROOVE_BOTTOM_NOT_FOUND          = 4,
  GROOVE_WALL_CENTROIDS_NOT_FOUND  = 5,
  MISSING_WEDGE_HISTORY            = 8,
  INVALID_SNAKE                    = 104,
  INVALID_WALL_HEIGHT_DIFFERENCE   = 105,
  SURFACE_ANGLE_TOLERANCE_EXCEEDED = 107,
  JOINT_WIDTH_OUT_OF_TOLERANCE     = 108,
  TWO_WALLS_NOT_FOUND              = 109,
  FAULTY_APPROXIMATION_DATA        = 110,
};

auto JointModelErrorCodeToString(JointModelErrorCode error_code) -> std::string;
auto JointModelErrorCodeToSnakeCaseString(JointModelErrorCode error_code) -> std::string;

struct JointProperties {
  double upper_joint_width;        ///< Upper width of the joint. Unit mm.
  double left_max_surface_angle;   ///< Left surface angle. Unit rad.
  double right_max_surface_angle;  ///< Right surface angle. Unit rad.
  double left_joint_angle;         ///< Angle of left joint. Unit rad.
  double right_joint_angle;        ///< Angle of right joint. Unit rad.
  double groove_depth;             ///< Depth of joint. Unit mm.

  double upper_joint_width_tolerance;  ///< Tolerance for upper joint width. Unit mm.
  double surface_angle_tolerance;      ///< Tolerance for surface angles. Unit rad.
  double groove_angle_tolerance;       ///< Tolerance for joint angles. Unit rad.

  double offset_distance;  ///< Edge offset for corner ABW points height extraction
};

inline auto ToYaml(const JointProperties& data) -> std::string {
  return "\nvalid: true\nupper_joint_width: " + fmt::format("{:.1f}", data.upper_joint_width) +
         "\ngroove_depth: " + fmt::format("{:.1f}", data.groove_depth) +
         "\nleft_joint_angle: " + fmt::to_string(data.left_joint_angle) +
         "\nright_joint_angle: " + fmt::to_string(data.right_joint_angle) +
         "\nleft_max_surface_angle: " + fmt::to_string(data.left_max_surface_angle) +
         "\nright_max_surface_angle: " + fmt::to_string(data.right_max_surface_angle) + "\n";
}

struct LineSegment {
  double k;
  double m;
  struct {
    double min;
    double max;
  } x_limits;
  struct {
    double min;
    double max;
  } y_limits;
  double theta;
  std::vector<int> inliers_indices;
  struct {
    Eigen::RowVectorXd x;
    Eigen::RowVectorXd y;
  } inliers;

  LineSegment() = delete;

  explicit LineSegment(double k, double m) : k(k), m(m) {}

  explicit LineSegment(Point p, double angle) : k(tan(angle)), m(p.y - tan(angle) * p.x) {}

  explicit LineSegment(Eigen::Index number_of_inliers) : k(0.0), m(0.0), x_limits({0.0, 0.0}), theta(0.0) {
    inliers_indices = std::vector<int>(number_of_inliers);
    inliers         = {Eigen::RowVectorXd(number_of_inliers), Eigen::RowVectorXd(number_of_inliers)};
  }

  [[nodiscard]] auto ToString() const -> std::string {
    std::stringstream stream;
    stream << *this;
    return stream.str();
  }

  friend auto operator<<(std::ostream& output_stream, const LineSegment& line_model) -> std::ostream& {
    output_stream << "k: " << line_model.k << '\n';
    output_stream << "m: " << line_model.m << '\n';
    output_stream << "theta: " << line_model.theta << '\n';
    output_stream << "min_x: " << line_model.x_limits.min << '\n';
    output_stream << "max_x: " << line_model.x_limits.max << '\n';
    output_stream << "min_y: " << line_model.y_limits.min << '\n';
    output_stream << "max_y: " << line_model.y_limits.max << '\n';
    output_stream << "num inliers: " << line_model.inliers.x.size() << '\n';
    return output_stream;
  }

  auto intersection(const LineSegment& other_line) const -> std::optional<std::tuple<double, double>> {
    auto k1 = k;
    auto k2 = other_line.k;
    auto m1 = m;
    auto m2 = other_line.m;

    if (k1 == k2) {
      return std::nullopt;
    }
    auto x = (m2 - m1) / (k1 - k2);
    auto y = k1 * x + m1;
    return {
        {x, y}
    };
  }

  auto TranslatedHorizontally(double distance) const -> LineSegment { return LineSegment{k, m - k * distance}; }

  auto ValueAt(float x) const -> float { return k * x + m; }
};

struct JointProfile {
  ABWPoints points = {
      Point{0, 0},
      Point{0, 0},
      Point{0, 0},
      Point{0, 0},
      Point{0, 0},
      Point{0, 0},
      Point{0, 0}
  };
  std::tuple<int, int> vertical_limits        = {0, 0};
  std::tuple<int, int> horizontal_limits      = {0, 0};
  std::optional<double> suggested_gain_change = std::nullopt;
  bool approximation_used                     = false;
  auto LeftDepth() const -> double { return points[0].y - points[1].y; };
  auto RightDepth() const -> double { return points[6].y - points[5].y; };
};

class JointModel {
 public:
  JointModel(const JointProperties& properties, image::CameraModelPtr camera_model)
      : properties_(properties), camera_model_(std::move(camera_model)) {};

  JointModel(const JointModel&)                        = delete;
  auto operator=(const JointModel&) -> JointModel&     = delete;
  JointModel(JointModel&&) noexcept                    = delete;
  auto operator=(JointModel&&) noexcept -> JointModel& = delete;

  virtual ~JointModel() = default;

  /**
   * Parses a matrix that represents the joint, the matrix that is parsed should contain three rows:
   *
   * joint = [x ; y ; z];
   *
   * Where x, y, z are the coordinates of the laser line on the object.
   *
   * @param joint
   * @return A JointProfile
   */
  virtual auto Parse(image::Image& image, std::optional<JointProfile> median_profile,
                     std::optional<JointProperties> updated_properties, bool use_approximation,
                     std::optional<std::tuple<double, double>> abw0_abw6_horizontal)
      -> std::expected<std::tuple<JointProfile, image::WorkspaceCoordinates, uint64_t, uint64_t>,
                       JointModelErrorCode> = 0;

  auto WorkspaceToImage(const image::WorkspaceCoordinates& workspace_coordinates, int vertical_crop_offset, int horizontal_crop_offset) const
      -> boost::outcome_v2::result<image::PlaneCoordinates> {
    return camera_model_->WorkspaceToImage(workspace_coordinates, vertical_crop_offset, horizontal_crop_offset);
  };

  static auto FitPoints(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>& x_and_y,
                        double residual_threshold) -> LineSegment;
  static auto FitPoints(const Eigen::RowVectorXd& x, const Eigen::RowVectorXd& y, double residual_threshold)
      -> LineSegment;

  /**
   *Converts the angle of the laser line to the angle orthogonal to the weld object center.
   *
   * @param angle The groove wall angle of the laser line in the laser plane.
   * @return The groove wall angle relative to the surface normal of the weld object.
   */
  static auto LPCSToWeldObjectAngle(double angle) -> double;

  /**
   *Converts the angle of the groove wall to the angle in the laser plane.
   *
   * @param angle  The groove wall angle relative to the surface normal of the weld object.
   * @return The groove wall angle relative to the surface normal of theweld object.
   */
  static auto LPCSFromWeldObjectAngle(double angle) -> double;

 protected:
  JointProperties properties_;
  image::CameraModelPtr camera_model_;
};

using JointModelPtr = std::unique_ptr<JointModel>;

}  // namespace scanner::joint_model

namespace std {
template <>
struct is_error_code_enum<scanner::joint_model::JointModelErrorCode> : true_type {};
}  // namespace std
