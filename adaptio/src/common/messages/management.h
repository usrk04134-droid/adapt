#pragma once

#include <cstdint>

namespace common::msg::management {

struct TrackingStart {
  enum class Metadata : uint32_t { MESSAGE_ID = 0x05000020 };
  uint32_t joint_tracking_mode{};
  float horizontal_offset{};
  float vertical_offset{};
};

struct TrackingUpdate {
  enum class Metadata : uint32_t { MESSAGE_ID = 0x05000021 };
  uint32_t joint_tracking_mode{};
  float horizontal_offset{};
  float vertical_offset{};
};

struct NotifyHandoverToManual {
  enum class Metadata : uint32_t { MESSAGE_ID = 0x05000022 };
};

struct TrackingStoppedGrooveDataTimeout {
  enum class Metadata : uint32_t { MESSAGE_ID = 0x05000023 };
};

struct ScannerError {
  enum class Metadata : uint32_t { MESSAGE_ID = 0x05000024 };
};

struct ABPStart {
  enum class Metadata : uint32_t { MESSAGE_ID = 0x05000025 };
};

struct ABPStop {
  enum class Metadata : uint32_t { MESSAGE_ID = 0x05000026 };
};

struct SubscribeReadyState {
  enum class Metadata : uint32_t { MESSAGE_ID = 0x05000027 };
};

struct ReadyState {
  enum class Metadata : uint32_t { MESSAGE_ID = 0x05000028 };

  enum class State : uint32_t {
    NOT_READY,
    NOT_READY_AUTO_CAL_MOVE,
    TRACKING_READY,
    ABP_READY,
    ABP_CAP_READY,
    ABP_AND_ABP_CAP_READY
  } state{ReadyState::State::NOT_READY};
};

struct Stop {
  enum class Metadata : uint32_t { MESSAGE_ID = 0x05000029 };
};

struct Shutdown {
  enum class Metadata : uint32_t { MESSAGE_ID = 0x05000030 };
};

struct ABPCapStart {
  enum class Metadata : uint32_t { MESSAGE_ID = 0x05000032 };
};

struct ABPCapStop {
  enum class Metadata : uint32_t { MESSAGE_ID = 0x05000033 };
};

struct GracefulStop {
  enum class Metadata : uint32_t { MESSAGE_ID = 0x05000034 };
};

}  // namespace common::msg::management
