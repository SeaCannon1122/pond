project(waveshare-pond_bundle)

add_library(waveshare-pond_bundle SHARED
  bundle.cpp
  ddsm115_driver.cpp
  DDSM115CMD.cpp
)

pond_module_common_setup(waveshare-pond_bundle)