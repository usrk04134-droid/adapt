#pragma once

#include <fmt/format.h>
#include <SQLiteCpp/Database.h>

#include <cstdint>
#include <functional>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

namespace weld_system {
struct WeldSystemSettings;
}  // namespace weld_system

namespace weld_control {

class WeldProcessParameters {
 public:
  WeldProcessParameters() = default;

  enum class Method { INVALID, DC, DC_NEG, AC };
  enum class RegulationType { INVALID, CW, CA, CC };
  enum class StartType { INVALID, DIRECT, SCRATCH };

  auto Id() const -> int;
  auto Name() const -> std::string;
  auto MethodValue() const -> Method;
  auto RegulationTypeValue() const -> RegulationType;
  auto StartAdjust() const -> uint32_t;
  auto StartTypeValue() const -> StartType;
  auto Voltage() const -> float;
  auto Current() const -> float;
  auto WireSpeed() const -> float;
  auto IceWireSpeed() const -> float;
  auto AcFrequency() const -> float;
  auto AcOffset() const -> float;
  auto AcPhaseShift() const -> float;
  auto CraterFillTime() const -> float;
  auto BurnBackTime() const -> float;

  void SetId(int);
  void SetName(std::string);
  void SetMethod(Method);
  void SetRegulationType(RegulationType);
  void SetStartAdjust(uint32_t);
  void SetStartType(StartType);
  void SetVoltage(float);
  void SetCurrent(float);
  void SetWireSpeed(float);
  void SetIceWireSpeed(float);
  void SetAcFrequency(float);
  void SetAcOffset(float);
  void SetAcPhaseShift(float);
  void SetCraterFillTime(float);
  void SetBurnBackTime(float);

  auto IsValid() const -> bool;
  auto ToString() const -> std::string;
  auto ToJson() const -> nlohmann::json;
  static auto FromJson(const nlohmann::json&) -> std::optional<WeldProcessParameters>;
  auto ToWeldSystemSettings() const -> std::optional<weld_system::WeldSystemSettings>;

  static void CreateTable(SQLite::Database*);
  static auto StoreFn() -> std::function<bool(SQLite::Database*, const WeldProcessParameters&)>;
  static auto RemoveFn() -> std::function<bool(SQLite::Database*, int)>;
  static auto GetAllFn() -> std::function<std::vector<WeldProcessParameters>(SQLite::Database*)>;

 private:
  static auto MethodToString(Method) -> std::string;
  static auto RegulationTypeToString(RegulationType) -> std::string;
  static auto StartTypeToString(StartType) -> std::string;
  static auto MethodFromString(const std::string&) -> Method;
  static auto RegulationTypeFromString(const std::string&) -> RegulationType;
  static auto StartTypeFromString(const std::string&) -> StartType;

  int id_{};
  std::string name_;
  Method method_{Method::INVALID};
  RegulationType regulation_type_{RegulationType::INVALID};
  uint32_t start_adjust_{};
  StartType start_type_{StartType::INVALID};
  float voltage_{};
  float current_{};
  float wire_speed_{};
  float ice_wire_speed_{};
  float ac_frequency_{};
  float ac_offset_{};
  float ac_phase_shift_{};
  float crater_fill_time_{};
  float burn_back_time_{};
};

}  // namespace weld_control
