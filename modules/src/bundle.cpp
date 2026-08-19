#define POND_MODULE_CPP_MAKE_IMPLEMENTATION
#include <pond/pond.hpp>

EXTERN_POND_MODULE(GstServer);
EXTERN_POND_MODULE(RealsenseCamera);
EXTERN_POND_MODULE(V4L2Camera);
EXTERN_POND_MODULE(DummyCamera);
EXTERN_POND_MODULE(DepthaiCamera);
//EXTERN_POND_MODULE(OrbSlam3Module);
//EXTERN_POND_MODULE(Ros2Bridge);

POND_BUNDLE_DECLARE(
    "All the modules for quac", 
    5,
    POND_MODULE(GstServer),
    POND_MODULE(RealsenseCamera),
    POND_MODULE(V4L2Camera),
    POND_MODULE(DummyCamera),
    POND_MODULE(DepthaiCamera)
    //POND_MODULE(OrbSlam3Module),
    //POND_MODULE(Ros2Bridge)
)