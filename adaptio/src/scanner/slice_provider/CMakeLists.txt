add_subdirectory(test)

target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    slice_provider.h
    circular_joint_buffer.h
    src/slice_provider_impl.h

    src/circular_joint_buffer.cc
    src/slice_provider_impl.cc
)
