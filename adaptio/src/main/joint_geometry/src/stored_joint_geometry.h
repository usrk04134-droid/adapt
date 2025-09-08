#pragma once

#include <fmt/format.h>
#include <SQLiteCpp/Database.h>

#include <cstdint>
#include <functional>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

namespace joint_geometry {

class StoredJointGeometry {
 public:
  StoredJointGeometry() = default;

  auto Id() const -> int;
  auto Name() const -> std::string;
  auto UpperJointWidth() const -> double;
  auto GrooveDepth() const -> double;
  auto LeftJointAngle() const -> double;
  auto RightJointAngle() const -> double;
  auto LeftMaxSurfaceAngle() const -> double;
  auto RightMaxSurfaceAngle() const -> double;

  void SetId(int);
  void SetName(std::string);
  void SetUpperJointWidth(double);
  void SetGrooveDepth(double);
  void SetLeftJointAngle(double);
  void SetRightJointAngle(double);
  void SetLeftMaxSurfaceAngle(double);
  void SetRightMaxSurfaceAngle(double);

  auto IsValid() const -> bool;
  auto ToJson() const -> nlohmann::json;
  static auto FromJson(const nlohmann::json&) -> std::optional<StoredJointGeometry>;
  auto ToString() const -> std::string;

  static void CreateTable(SQLite::Database*);
  static auto StoreFn() -> std::function<bool(SQLite::Database*, const StoredJointGeometry&)>;
  static auto RemoveFn() -> std::function<bool(SQLite::Database*, int)>;
  static auto GetAllFn() -> std::function<std::vector<StoredJointGeometry>(SQLite::Database*)>;

 private:
  int id_{};
  std::string name_;
  double upper_joint_width_mm_{};
  double groove_depth_mm_{};
  double left_joint_angle_rad_{};
  double right_joint_angle_rad_{};
  double left_max_surface_angle_rad_{};
  double right_max_surface_angle_rad_{};
};

};  // namespace joint_geometry
