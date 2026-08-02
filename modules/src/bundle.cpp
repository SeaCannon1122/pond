#include <pond/pond.h>

EXTERN_POND_MODULE(GstServer);
EXTERN_POND_MODULE(RealsenseDriver);
EXTERN_POND_MODULE(UsbCamStreamer);

POND_BUNDLE_DECLARE(
    "All the modules for quac", 
    3, 
    POND_MODULE(GstServer), 
    POND_MODULE(RealsenseDriver),
    POND_MODULE(UsbCamStreamer)
)