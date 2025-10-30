add_subdirectory(test)

target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    camera_model.h
    image.h
    image_builder.h
    image_types.h
    image_utilities.h
    tiff_handler.h
    tiff_handler_impl.h
    tilted_perspective_camera.h

    src/camera_model.cc
    src/image.cc
    src/image_builder.cc
    src/image_utilities.cc
    src/tiff_handler_impl.cc
    src/tilted_perspective_camera.cc
)
