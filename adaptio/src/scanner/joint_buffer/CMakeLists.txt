target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    joint_buffer.h
    circular_joint_buffer.h
    src/single_joint_buffer.h

    src/circular_joint_buffer.cc
    src/single_joint_buffer.cc
)
