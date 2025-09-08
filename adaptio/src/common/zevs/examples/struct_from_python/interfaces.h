#pragma once

#include <cstdint>

namespace test_if {

struct TestS1 {
  enum class Metadata : uint32_t { MESSAGE_ID = 0x03000100 };
  int i1;
  float f1;
  bool b1;
  uint32_t u1;
};

struct TestS2 {
  enum class Metadata : uint32_t { MESSAGE_ID = 0x03000101 };
  double d1;
  int64_t i1;
};

}  // namespace test_if
