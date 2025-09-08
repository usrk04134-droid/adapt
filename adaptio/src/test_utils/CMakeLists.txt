set(FILES
  reporters.h
  testlog.h
)

target_sources(${UNIT_TESTS_APPLICATION}
    PRIVATE

    ${FILES}
)

target_sources(${BLOCK_TESTS_APPLICATION}
    PRIVATE

    ${FILES}
)
