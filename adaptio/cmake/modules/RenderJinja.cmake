function(render_jinja NAME TARGETS OUTPUT TEMPLATE DATA)
    if(NOT DEFINED ENV{PN_INTERFACE})
        message(FATAL_ERROR "PN_INTERFACE environment variable is not set!")
    endif()

    set(RENDER_JINJA "$ENV{PN_INTERFACE}/bin/render-jinja")

    add_custom_command(
        OUTPUT ${OUTPUT}
        COMMAND ${RENDER_JINJA} ${OUTPUT} ${TEMPLATE} ${DATA}
        DEPENDS ${TEMPLATE} ${DATA}
        COMMENT "Generating ${OUTPUT} from ${TEMPLATE} using ${DATA}"
        VERBATIM
    )

    add_custom_target(
        ${NAME}
        DEPENDS ${OUTPUT}
    )

    foreach(TARGET ${TARGETS})
        add_dependencies(${TARGET} ${NAME})
    endforeach()
endfunction()

