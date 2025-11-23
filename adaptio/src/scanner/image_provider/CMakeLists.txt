add_subdirectory(test)

target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    image_provider.h
    image_provider_configuration.h
    src/camera_simulation.h
    src/basler_camera.h
    src/buffered_channel.h

    src/camera_simulation.cc

    ../../../build/basler_camera_with_pfs.cc
)
