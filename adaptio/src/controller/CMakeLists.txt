include(RenderJinja)
set(CONTROLLER_GENERATED_PATH ${CMAKE_BINARY_DIR}/generated/controller)

file(MAKE_DIRECTORY ${CONTROLLER_GENERATED_PATH})

set(ASSETS_GENERATED_PATH ${CMAKE_BINARY_DIR}/generated/assets)

file(MAKE_DIRECTORY ${ASSETS_GENERATED_PATH})

MESSAGE(STATUS "Generating files in  ${CONTROLLER_GENERATED_PATH}")

## Generate controller.h
render_jinja(
        GenerateControllerH
        "${ADAPTIO_APPLICATION};${UNIT_TESTS_APPLICATION};${BLOCK_TESTS_APPLICATION};${TEST_IMAGES_APPLICATION}"
        ${CONTROLLER_GENERATED_PATH}/controller.h
        ${CMAKE_CURRENT_SOURCE_DIR}/controller.h.jinja2
        $ENV{PN_INTERFACE}/etc/pn-interface.yaml
)

## Generate controller_data.h
render_jinja(
        GenerateControllerDataH
        "${ADAPTIO_APPLICATION};${UNIT_TESTS_APPLICATION};${BLOCK_TESTS_APPLICATION};${TEST_IMAGES_APPLICATION}"
        ${CONTROLLER_GENERATED_PATH}/controller_data.h
        ${CMAKE_CURRENT_SOURCE_DIR}/controller_data.h.jinja2
        $ENV{PN_INTERFACE}/etc/pn-interface.yaml
)

target_include_directories(${ADAPTIO_APPLICATION} PRIVATE ${CMAKE_BINARY_DIR}/generated/)
target_include_directories(${UNIT_TESTS_APPLICATION} PRIVATE ${CMAKE_BINARY_DIR}/generated/)
target_include_directories(${BLOCK_TESTS_APPLICATION} PRIVATE ${CMAKE_BINARY_DIR}/generated/)
target_include_directories(${TEST_IMAGES_APPLICATION} PRIVATE ${CMAKE_BINARY_DIR}/generated/)

add_subdirectory(tests)

target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    ${CONTROLLER_GENERATED_PATH}/controller.h
    ${CONTROLLER_GENERATED_PATH}/controller_data.h
    controller_factory.h
    controller_messenger.h
    controller_configuration.h
    management_client.h
    kinematics_server.h
    kinematics_server_impl.h
    weld_axis_buffer.h
    slide_position_buffer.h
    axis_output_plc_adapter.h
    weld_system_server.h
    weld_system_server_impl.h
    weld_axis_filter.h

    controller.cc
    controller_factory.cc
    controller_messenger.cc
    management_client.cc
    kinematics_server_impl.cc
    weld_axis_buffer.cc
    slide_position_buffer.cc
    axis_output_plc_adapter.cc
    weld_system_server_impl.cc
    weld_axis_filter.cc

)

add_subdirectory(simulation)
add_subdirectory(pn_driver)
