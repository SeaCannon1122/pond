project(detection-pond_bundle)

add_library(quirc quirc/quirc.c quirc/decode.c quirc/identify.c quirc/version_db.c)
target_compile_definitions(quirc PRIVATE QUIRC_MAX_REGIONS=65534)

add_library(detection-pond_bundle SHARED
  bundle.cpp
  qrcode_detection_service.cpp
  landoltc_detection_service.cpp
)

target_link_libraries(detection-pond_bundle quirc)

pond_module_common_setup(detection-pond_bundle)