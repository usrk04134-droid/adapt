#pragma once

#include <cstdint>

namespace common::msg::kinematics {

const uint32_t KINEMATICS_BASE_ID = 0x06000000;

struct GetSlidesPosition {
  enum class Metadata : uint32_t { MESSAGE_ID = KINEMATICS_BASE_ID + 1 };
  std::uint64_t time_stamp{}; /* 0 = latest sample */
  uint32_t client_id{};
};

struct GetSlidesPositionRsp {
  enum class Metadata : uint32_t { MESSAGE_ID = KINEMATICS_BASE_ID + 2 };
  uint32_t client_id{};
  std::uint64_t time_stamp{};
  double horizontal{};
  double vertical{};
};

struct SetSlidesPosition {
  enum class Metadata : uint32_t { MESSAGE_ID = KINEMATICS_BASE_ID + 3 };
  double horizontal{};
  double vertical{};
  double horizontal_lin_velocity{}; /* mm/sec */
  double vertical_velocity{};       /* mm/sec */
};

struct GetSlidesStatus {
  enum class Metadata : uint32_t { MESSAGE_ID = KINEMATICS_BASE_ID + 4 };
  uint32_t client_id{};
};

struct GetSlidesStatusRsp {
  enum class Metadata : uint32_t { MESSAGE_ID = KINEMATICS_BASE_ID + 5 };
  uint32_t client_id{};
  bool horizontal_in_position{};
  bool vertical_in_position{};
};

struct GetWeldAxisData {
  enum class Metadata : uint32_t { MESSAGE_ID = KINEMATICS_BASE_ID + 6 };
  uint32_t client_id{};
  std::uint64_t time_stamp{}; /* 0 = latest sample */
};

struct GetWeldAxisDataRsp {
  enum class Metadata : uint32_t { MESSAGE_ID = KINEMATICS_BASE_ID + 7 };
  uint32_t client_id{};
  std::uint64_t time_stamp{};
  double position{};
  double velocity{};
  double radius{};
  double linear_object_distance{};
};

struct Release {
  enum class Metadata : uint32_t { MESSAGE_ID = KINEMATICS_BASE_ID + 8 };
};

struct SetWeldAxisData {
  enum class Metadata : uint32_t { MESSAGE_ID = KINEMATICS_BASE_ID + 9 };
  double velocity{}; /* mm/sec */
};

struct SubscribeStateChanges {
  enum class Metadata : uint32_t { MESSAGE_ID = KINEMATICS_BASE_ID + 10 };
};

struct UnSubscribeStateChanges {
  enum class Metadata : uint32_t { MESSAGE_ID = KINEMATICS_BASE_ID + 11 };
};

struct StateChange {
  enum class Metadata : uint32_t { MESSAGE_ID = KINEMATICS_BASE_ID + 12 };
  enum class State : uint32_t {
    INIT,
    HOMED,
  } weld_axis_state{State::INIT};
};

struct GetEdgePosition {
  enum class Metadata : uint32_t { MESSAGE_ID = KINEMATICS_BASE_ID + 13 };
  uint32_t client_id{};
};

struct GetEdgePositionRsp {
  enum class Metadata : uint32_t { MESSAGE_ID = KINEMATICS_BASE_ID + 14 };
  uint32_t client_id{};
  double position{};
};

struct EdgeStateChange {
  enum class Metadata : uint32_t { MESSAGE_ID = KINEMATICS_BASE_ID + 15 };
  enum class State : uint32_t {
    NOT_AVAILABLE,
    AVAILABLE,
  } edge_state{State::NOT_AVAILABLE};
};

}  // namespace common::msg::kinematics
