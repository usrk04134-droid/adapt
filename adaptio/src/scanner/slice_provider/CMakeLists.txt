add_subdirectory(test)

target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    slice_provider.h
    src/circular_joint_buffer.h
    src/slice_provider_impl.h

    src/slice_provider_impl.cc
    src/circular_joint_buffer.cc
)
