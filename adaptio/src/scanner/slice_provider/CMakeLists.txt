add_subdirectory(test)

target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    slice_provider.h
    slice_provider_impl.h

    slice_provider_impl.cc
)
