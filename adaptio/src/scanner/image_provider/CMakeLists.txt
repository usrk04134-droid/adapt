add_subdirectory(test)

target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    basler_camera.h
    image_provider.h
    image_provider_configuration.h
    camera_simulation.h
    src/buffered_channel.h

    src/camera_simulation.cc

    ../../../build/basler_camera_with_pfs.cc
)
