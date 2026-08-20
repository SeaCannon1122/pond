#include "pond/pond.h"
#define POND_MODULE_CPP_MAKE_IMPLEMENTATION
#include <pond/pond.hpp>

EXTERN_POND_MODULE(GstServer);
EXTERN_POND_MODULE(RealsenseCamera);
EXTERN_POND_MODULE(V4L2Camera);
EXTERN_POND_MODULE(DummyCamera);
EXTERN_POND_MODULE(DepthaiCamera);
EXTERN_POND_MODULE(OrbSlam3);
EXTERN_POND_MODULE(RTABMap);
EXTERN_POND_MODULE(DepthColorizer);
EXTERN_POND_MODULE(Ros2Bridge);
EXTERN_POND_MODULE(DDSM115Driver);

POND_BUNDLE_DECLARE(
    "All the modules for quac", 
    10,
    POND_MODULE(GstServer),
    POND_MODULE(RealsenseCamera),
    POND_MODULE(V4L2Camera),
    POND_MODULE(DummyCamera),
    POND_MODULE(DepthaiCamera),
    POND_MODULE(OrbSlam3),
    POND_MODULE(RTABMap),
    POND_MODULE(DepthColorizer),
    POND_MODULE(Ros2Bridge),
    POND_MODULE(DDSM115Driver)
)