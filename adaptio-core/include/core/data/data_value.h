#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <variant>

namespace core::data {

enum DataValueType {
  BOOL,
  INT64_T,
  UINT64_T,
  DOUBLE,
  STRING,
  MATRIX,
};

struct Matrix {
  size_t rows;
  size_t columns;
  std::shared_ptr<double[]> data;
};

/**
 * Class that represents a data value received from a dataprovider.
 */
class DataValue {
 public:
  explicit DataValue() = delete;
  explicit DataValue(bool value);
  explicit DataValue(int16_t value);
  explicit DataValue(int32_t value);
  explicit DataValue(int64_t value);
  explicit DataValue(uint16_t value);
  explicit DataValue(uint32_t value);
  explicit DataValue(uint64_t value);
  explicit DataValue(float value);
  explicit DataValue(double value);
  explicit DataValue(std::string value);
  explicit DataValue(Matrix value);

  template <typename T>
  auto Value() -> std::optional<T> {
    try {
      return std::get<T>(value_);
    } catch (const std::bad_variant_access& ex) {
      return std::nullopt;
    }
  }

  template <typename T>
  [[nodiscard]] auto Value() const -> std::optional<T> {
    try {
      return std::get<T>(value_);
    } catch (const std::bad_variant_access& ex) {
      return std::nullopt;
    }
  }

  template <typename T>
  static auto Dummy() -> T {
    auto dummy = T();
    return dummy;
  }

  auto operator<=>(const DataValue&) const;
  auto operator==(const DataValue&) const -> bool;
  auto operator!=(const DataValue&) const -> bool;

  static auto FromString(const std::string& value_string) -> std::optional<DataValue>;

  /**
   *
   * @return The data value as a string containing only the value in string
   * representation
   */
  [[nodiscard]] auto ToValueString() const -> std::string;

  /**
   *
   * @return The data value as a string containing the type of the value and the
   * actual value in string representation
   */
  [[nodiscard]] auto ToString() const -> std::string;

 private:
  std::variant<bool, int64_t, uint64_t, double, std::string, Matrix> value_;
  DataValueType type_;
};

}  // namespace core::data
