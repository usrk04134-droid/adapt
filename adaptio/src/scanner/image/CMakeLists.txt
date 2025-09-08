add_subdirectory(test)

target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    camera_model.h
    image.h
    image_builder.h
    image_data.h
    image_types.h
    image_utilities.h
    tiff_handler.h
    tiff_handler_impl.h
    tilted_perspective_camera.h

    camera_model.cc
    image.cc
    image_builder.cc
    image_data.cc
    image_utilities.cc
    tiff_handler_impl.cc
    tilted_perspective_camera.cc
)
