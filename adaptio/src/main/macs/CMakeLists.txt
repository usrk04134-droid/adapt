add_subdirectory(test)

target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    macs_point.h
    macs_slice.h
    macs_groove.h
    macs_math.h
)
