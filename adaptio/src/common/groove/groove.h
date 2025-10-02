#pragma once

#include <fmt/core.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <nlohmann/json.hpp>
#include <numbers>
#include <vector>

#include "common/logging/application_log.h"
#include "common/groove/math.h"
#include "common/groove/point.h"

namespace common::groove {

auto const ABW_POINTS      = 7;
auto const ABW_UPPER_LEFT  = 0;
auto const ABW_LOWER_LEFT  = 1;
auto const ABW_LOWER_RIGHT = 5;
auto const ABW_UPPER_RIGHT = 6;

class Groove final {
 public:
  Groove()                    = default;
  Groove(const Groove& other) = default;
  Groove(Groove&&)            = default;

  explicit Groove(const std::array<Point, ABW_POINTS>& arr) : groove_(arr) {}
  explicit Groove(const std::vector<Point>& vec) {
    assert(vec.size() == ABW_POINTS);
    std::ranges::copy(vec, groove_.begin());
  }

  Groove(const Point& p0, const Point& p1, const Point& p2, const Point& p3, const Point& p4, const Point& p5,
         const Point& p6) {
    groove_ = {p0, p1, p2, p3, p4, p5, p6};
  }

  ~Groove() = default;

  auto operator=(const Groove&) -> Groove& = default;
  auto operator=(Groove&&) -> Groove&      = default;

  auto operator[](size_t index) -> Point& {
    assert(index < ABW_POINTS);
    return groove_.at(index);
  }

  auto operator[](size_t index) const -> const Point& {
    assert(index < ABW_POINTS);
    return groove_.at(index);
  }

  auto Area() const -> double { return PolygonArea({groove_.begin(), groove_.end()}); }

  auto LeftDepth() const -> double { return (groove_.at(ABW_UPPER_LEFT) - groove_.at(ABW_LOWER_LEFT)).vertical; }

  auto RightDepth() const -> double { return (groove_.at(ABW_UPPER_RIGHT) - groove_.at(ABW_LOWER_RIGHT)).vertical; }

  auto AvgDepth() const -> double { return (LeftDepth() + RightDepth()) / 2.0; }

  auto TopWidth() const -> double { return (groove_.at(ABW_UPPER_LEFT) - groove_.at(ABW_UPPER_RIGHT)).horizontal; }

  auto BottomWidth() const -> double { return (groove_.at(ABW_LOWER_LEFT) - groove_.at(ABW_LOWER_RIGHT)).horizontal; }

  auto TopSlope() const -> double {
    return (groove_.at(ABW_UPPER_LEFT) - groove_.at(ABW_UPPER_RIGHT)).vertical /
           (groove_.at(ABW_UPPER_RIGHT) - groove_.at(ABW_UPPER_LEFT)).horizontal;
  }

  auto BottomSlope() const -> double {
    return (groove_.at(ABW_LOWER_LEFT) - groove_.at(ABW_LOWER_RIGHT)).vertical /
           (groove_.at(ABW_LOWER_RIGHT) - groove_.at(ABW_LOWER_LEFT)).horizontal;
  }

  auto LeftWallAngle() const -> double {
    auto const val = groove_.at(ABW_UPPER_LEFT) - groove_.at(ABW_LOWER_LEFT);
    return std::atan2(val.horizontal, val.vertical);
  }

  auto RightWallAngle() const -> double {
    auto const val = groove_.at(ABW_LOWER_RIGHT) - groove_.at(ABW_UPPER_RIGHT);
    return (std::numbers::pi / 2) + std::atan2(val.vertical, val.horizontal);
  }

  auto ToString() const -> std::string {
    std::string str = "{";
    for (const auto& point : groove_) {
      str += fmt::format("{{{:.2f}, {:.2f}}},", point.horizontal, point.vertical);
    }
    str += "}";

    return str;
  }

  void Move(Point adjustment) { std::ranges::for_each(groove_, [adjustment](Point& point) { point += adjustment; }); }

  auto ToVector() const -> std::vector<Point> { return {groove_.begin(), groove_.end()}; }

  auto IsValid() const -> bool {
    auto valid = std::ranges::all_of(
        groove_, [](Point point) { return !std::isnan(point.horizontal) && !std::isnan(point.vertical); });
    valid &= groove_[ABW_LOWER_LEFT].horizontal > groove_[ABW_LOWER_RIGHT].horizontal;

    return valid;
  }

  auto ToJson() const -> nlohmann::json {
    auto json_groove = nlohmann::json::array();
    for (auto const& point : groove_) {
      nlohmann::json point_json = {
          {"horizontal", point.horizontal},
          {"vertical",   point.vertical  }
      };
      json_groove.push_back(point_json);
    }
    return json_groove;
  }

  static auto FromJson(const nlohmann::json& json_obj) -> std::optional<Groove> {
    try {
      if (!json_obj.is_array() || json_obj.size() != ABW_POINTS) {
        return std::nullopt;
      }

      Groove groove;
      for (size_t i = 0; i < ABW_POINTS; ++i) {
        auto const& point_json = json_obj[i];
        groove[i]              = {
            .horizontal = point_json.at("horizontal").get<double>(),
            .vertical   = point_json.at("vertical").get<double>(),
        };
      }

      return groove;
    } catch (const nlohmann::json::exception& e) {
      LOG_ERROR("Failed to parse Groove from JSON - exception: {}", e.what());
      return std::nullopt;
    }
  }

  using iterator       = Point*;
  using const_iterator = const Point*;

  auto begin() -> iterator { return groove_.data(); }
  auto end() -> iterator { return groove_.data() + groove_.size(); }
  auto begin() const -> const_iterator { return groove_.data(); }
  auto end() const -> const_iterator { return groove_.data() + groove_.size(); }
  auto cbegin() const -> const_iterator { return groove_.data(); }
  auto cend() const -> const_iterator { return groove_.data() + groove_.size(); }

 private:
  std::array<Point, ABW_POINTS> groove_{};
};

}  // namespace common::groove

