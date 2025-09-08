add_subdirectory(test)

target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    yaml.h

    yaml.cc
)
