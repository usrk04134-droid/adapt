target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
        web_hmi_server.h
        json_payload.h
        service.h
        service_mode_manager_impl.h
        service_mode_tracking.h

        web_hmi_server.cc
        service_mode_manager_impl.cc
        service_mode_tracking.cc
    )
