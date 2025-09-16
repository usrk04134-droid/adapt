target_sources(${ADAPTIO_APPLICATION}
    PRIVATE
    slice_translator.h
    slice_observer.h
    src/coordinates_translator.h
    slice_translator_service_v2.h
    model_extract.h
    model_config.h
    src/model_impl.h
    
    src/coordinates_translator.cc
    src/model_impl.cc
)
