add_subdirectory(test)

target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    event_codes.h
    event_handler.h
    src/event_types.h
    src/event_handler_impl.h

    src/event_handler_impl.cc
)
