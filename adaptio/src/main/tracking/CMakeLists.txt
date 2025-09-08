add_subdirectory(tests)

target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    tracking_manager.h
)

add_subdirectory(src)
