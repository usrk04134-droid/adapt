add_subdirectory(test)

target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    data_value.h

    data_value.cc
)
