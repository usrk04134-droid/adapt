include(RenderJinja)
set(PN_DRIVER_DATA_GENERATED_PATH ${CMAKE_BINARY_DIR}/generated/controller/pn_driver)

file(MAKE_DIRECTORY ${PN_DRIVER_DATA_GENERATED_PATH})

MESSAGE(STATUS "Generating files in  ${PN_DRIVER_DATA_GENERATED_PATH}")

# Generate pn_driver_data.h
render_jinja(
        GeneratePnDriverDataH
        "${ADAPTIO_APPLICATION};${UNIT_TESTS_APPLICATION};${BLOCK_TESTS_APPLICATION};${TEST_IMAGES_APPLICATION}"
        ${PN_DRIVER_DATA_GENERATED_PATH}/pn_driver_data.h
        ${CMAKE_CURRENT_SOURCE_DIR}/pn_driver_data.h.jinja2
        $ENV{PN_INTERFACE}/etc/pn-interface.yaml
)

# Generate pn_driver_data.cc
render_jinja(
        GeneratePnDriverDataCc
        "${ADAPTIO_APPLICATION};${UNIT_TESTS_APPLICATION};${BLOCK_TESTS_APPLICATION};${TEST_IMAGES_APPLICATION}"
        ${PN_DRIVER_DATA_GENERATED_PATH}/pn_driver_data.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/pn_driver_data.cc.jinja2
        $ENV{PN_INTERFACE}/etc/pn-interface.yaml
)

target_sources(${ADAPTIO_APPLICATION}
        PRIVATE
        ${PN_DRIVER_DATA_GENERATED_PATH}/pn_driver_data.h
        pn_driver.h
        pn_driver_callbacks.h

        ${PN_DRIVER_DATA_GENERATED_PATH}/pn_driver_data.cc
        pn_driver.cc
)

add_subdirectory(shared_src)
