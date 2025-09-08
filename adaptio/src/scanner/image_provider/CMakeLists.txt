add_subdirectory(simulation)
add_subdirectory(test)

target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    basler_camera.h
    image_provider.h
    image_provider_configuration.h
    buffered_channel.h

    ../../../build/basler_camera_with_pfs.cc
)
