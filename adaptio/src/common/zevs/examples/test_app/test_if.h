#pragma once

#include <cstdint>

namespace test_if {

struct A {
  enum class Metadata : uint32_t { MESSAGE_ID = 0x01000000 };
  uint32_t i1;
  uint32_t i2;
  char s1[100];
};

struct B {
  enum class Metadata : uint32_t { MESSAGE_ID = 0x01000001 };
  bool b1;
  uint32_t i1;
};

struct C {
  enum class Metadata : uint32_t { MESSAGE_ID = 0x010000002 };
  uint32_t i1;
  uint32_t i2;
};

}  // namespace test_if
