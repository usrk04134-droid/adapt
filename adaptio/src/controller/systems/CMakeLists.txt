add_subdirectory(tests)

target_sources(${ADAPTIO_APPLICATION}
        PRIVATE
        system.h
        heartbeat/heartbeat_system.h

        heartbeat/heartbeat_system.cc
)
