add_subdirectory(test)

target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    centroids.h
    filter.h
    lin_interp.h
    math.h
    math.h
    value.h

    filter.cc
    lin_interp.cc
)
