#pragma once

#include <cstdint>

namespace weld_system {

enum class WeldSystemId : uint32_t {
  INVALID = 0,
  ID1     = 1,
  ID2     = 2,
};

enum class WeldSystemState {
  INIT,
  READY_TO_START,
  IN_WELDING_SEQUENCE,
  ARCING,
  ERROR,
};

struct WeldSystemData {
  double voltage{0.0};
  double current{0.0};
  double wire_lin_velocity{0.0};
  double deposition_rate{0.0};
  double heat_input{0.0};
  bool twin_wire{false};
  double wire_diameter{0.0};
};

struct WeldSystemSettings {
  enum class Method : uint32_t {
    DC,
    DC_NEG,
    AC,
  } method{};

  enum class RegulationType : uint32_t {
    CW,
    CA,
    CC,
  } regulation_type{};

  uint32_t start_adjust{0};

  enum class StartType : uint32_t {
    DIRECT,
    SCRATCH,
  } start_type{};

  double voltage{0.0};
  double current{0.0};
  double wire_speed{0.0};
  double ice_wire_speed{0.0};
  double ac_frequency{0.0};
  double ac_offset{0.0};
  double ac_phase_shift{0.0};
  double crater_fill_time{0.0};
  double burn_back_time{0.0};
};

inline auto WeldSystemStateToString(WeldSystemState state) -> const char* {
  switch (state) {
    case WeldSystemState::INIT:
      return "init";
    case WeldSystemState::READY_TO_START:
      return "ready-to-start";
    case WeldSystemState::IN_WELDING_SEQUENCE:
      return "in-welding-sequence";
    case WeldSystemState::ARCING:
      return "arcing";
    case WeldSystemState::ERROR:
      return "error";
    default:
      break;
  }
  return "invalid-state";
};

inline auto WeldSystemIdToString(WeldSystemId id) -> const char* {
  switch (id) {
    case WeldSystemId::ID1:
      return "1";
    case WeldSystemId::ID2:
      return "2";
    case WeldSystemId::INVALID:
      break;
  };
  return "invalid";
}
}  // namespace weld_system
