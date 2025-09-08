#ifndef DOCTEST_CONFIG_DISABLE
#include "core/channels/buffered_channel.h"

#include <doctest/doctest.h>

#include <cstdint>

using core::channels::BufferedChannel;

TEST_SUITE("BufferedChannel tests") {
  TEST_CASE("Test get reader") {
    auto channel = BufferedChannel<bool>();

    auto reader = channel.GetReader();

    CHECK_NE(reader.get(), nullptr);
  }

  TEST_CASE("Test get writer") {
    auto channel = BufferedChannel<bool>();

    auto writer = channel.GetWriter();

    CHECK_NE(writer.get(), nullptr);
  }

  TEST_CASE("Test write/read in order") {
    auto channel = BufferedChannel<int32_t>();

    auto writer = channel.GetWriter();
    auto reader = channel.GetReader();

    writer->Write(1);
    writer->Write(2);
    writer->Write(3);

    CHECK_EQ(reader->IsEmpty(), false);

    CHECK_EQ(reader->Read(), 1);
    CHECK_EQ(reader->Read(), 2);
    CHECK_EQ(reader->Read(), 3);

    CHECK_EQ(reader->IsEmpty(), true);
  }
}
#endif
