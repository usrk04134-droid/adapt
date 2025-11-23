add_subdirectory(core)
add_subdirectory(image)
add_subdirectory(test)
add_subdirectory(image_provider)
add_subdirectory(joint_model)
add_subdirectory(joint_buffer)
add_subdirectory(image_logger)
add_subdirectory(slice_provider)

target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    scanner_application.h
    scanner_factory.h
    scanner_factory_impl.h

    scanner_application.cc
    scanner_factory_impl.cc
)
