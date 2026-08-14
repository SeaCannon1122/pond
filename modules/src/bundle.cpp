#define POND_MODULE_CPP_MAKE_IMPLEMENTATION
#include <pond/pond.hpp>

EXTERN_POND_MODULE(GstServer);
//EXTERN_POND_MODULE(RealsenseDriver);
EXTERN_POND_MODULE(UsbCamDriver);
//EXTERN_POND_MODULE(DepthaiDriver);
//EXTERN_POND_MODULE(OrbSlam3Module);
//EXTERN_POND_MODULE(Ros2Bridge);

POND_BUNDLE_DECLARE(
    "All the modules for quac", 
    2,
    POND_MODULE(GstServer), 
    //POND_MODULE(RealsenseDriver),
    POND_MODULE(UsbCamDriver),
    //POND_MODULE(DepthaiDriver),
    //POND_MODULE(OrbSlam3Module),
    //POND_MODULE(Ros2Bridge)
)