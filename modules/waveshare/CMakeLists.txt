project(waveshare-pond_bundle)

add_library(waveshare-pond_bundle SHARED
  bundle.cpp

  ddsm115_driver.cpp
  DDSM115CMD.cpp

  servo_driver.cpp
  sms_sts/SCS.cpp
  sms_sts/SCSerial.cpp
  sms_sts/SMS_STS.cpp
)

pond_module_common_setup(waveshare-pond_bundle)