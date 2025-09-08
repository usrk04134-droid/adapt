target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    joint_geometry.h
    joint_geometry_provider.h
    src/joint_geometry_provider_impl.h
    src/stored_joint_geometry.h

    src/joint_geometry_provider_impl.cc
    src/stored_joint_geometry.cc
    src/joint_geometry.cc
)
