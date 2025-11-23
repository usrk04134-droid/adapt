target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    scanner_calibration_configuration.h
    scanner_configuration.h
    scanner_types.h
    src/scanner.h
    src/scanner_impl.h
    src/scanner_server.h

    src/scanner_impl.cc
    src/scanner_server.cc
)
