
// NOLINTBEGIN(*-magic-numbers, *-optional-access)

#include <doctest/doctest.h>

#include "common/messages/management.h"
#include "helpers.h"

TEST_SUITE("Shutdown") {
  TEST_CASE("basic") {
    TestFixture fixture;
    fixture.StartApplication();

    // Shutdown
    fixture.Management()->Dispatch(common::msg::management::Shutdown{});

    CHECK(fixture.Sut()->InShutdown());
  }
}

// NOLINTEND(*-magic-numbers, *-optional-access)
