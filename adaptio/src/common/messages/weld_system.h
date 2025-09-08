#pragma once

#include <cstdint>

namespace common::msg::weld_system {

const uint32_t WELD_SYSTEM_BASE_ID = 0x07000000;

struct GetWeldSystemData {
  enum class Metadata : uint32_t { MESSAGE_ID = WELD_SYSTEM_BASE_ID + 1 };
  uint32_t transaction_id;

  uint32_t index; /* weld-system index starting at 1 */
};

struct GetWeldSystemDataRsp {
  enum class Metadata : uint32_t { MESSAGE_ID = WELD_SYSTEM_BASE_ID + 2 };
  uint32_t transaction_id{0};

  float voltage{0.0};
  float current{0.0};
  float wire_lin_velocity{0.0};  // mm/sec
  float deposition_rate{0.0};    // kg/h
  float heat_input{0.0};
  bool twin_wire{false};
  float wire_diameter{0.0};
};

struct SetWeldSystemSettings {
  enum class Metadata : uint32_t { MESSAGE_ID = WELD_SYSTEM_BASE_ID + 3 };

  uint32_t index{0}; /* weld-system index starting at 1 */

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

  float voltage{0.0};
  float current{0.0};
  float wire_speed{0.0};
  float ice_wire_speed{0.0};
  float ac_frequency{0.0};
  float ac_offset{0.0};
  float ac_phase_shift{0.0};
  float crater_fill_time{0.0};
  float burn_back_time{0.0};
};

struct SubscribeWeldSystemStateChanges {
  enum class Metadata : uint32_t { MESSAGE_ID = WELD_SYSTEM_BASE_ID + 4 };
};

struct UnSubscribeWeldSystemStateChanges {
  enum class Metadata : uint32_t { MESSAGE_ID = WELD_SYSTEM_BASE_ID + 5 };
};

struct OnWeldSystemStateChange {
  enum class Metadata : uint32_t { MESSAGE_ID = WELD_SYSTEM_BASE_ID + 6 };

  uint32_t index; /* weld-system index starting at 1 */
  enum class State : uint32_t {
    INIT,
    READY_TO_START,
    IN_WELDING_SEQUENCE,
    ARCING,
    ERROR,
  } state;
};

}  // namespace common::msg::weld_system
