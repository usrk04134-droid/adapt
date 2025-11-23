add_subdirectory(test)

target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    slice_provider.h
    src/slice_provider_impl.h

    src/slice_provider_impl.cc
)
