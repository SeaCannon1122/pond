#define POND_MODULE_CPP_MAKE_IMPLEMENTATION
#include <pond/pond.hpp>

EXTERN_POND_MODULE(V4L2Camera);
EXTERN_POND_MODULE(DummyCamera);
EXTERN_POND_MODULE(DepthColorizer);

POND_BUNDLE_DECLARE(
    "Camera modules", 
    3,
    POND_MODULE(V4L2Camera),
    POND_MODULE(DummyCamera),
    POND_MODULE(DepthColorizer),
)