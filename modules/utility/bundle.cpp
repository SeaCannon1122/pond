#include "pond/pond.h"
#define POND_MODULE_CPP_MAKE_IMPLEMENTATION
#include <pond/pond.hpp>

EXTERN_POND_MODULE(FrameTimer);
EXTERN_POND_MODULE(DummyCamera);
EXTERN_POND_MODULE(DepthColorizer);
EXTERN_POND_MODULE(MotorTester);
EXTERN_POND_MODULE(DummyMotor);

POND_BUNDLE_DECLARE(
    "Camera modules", 
    5,
    POND_MODULE(FrameTimer),
    POND_MODULE(DummyCamera),
    POND_MODULE(DepthColorizer),
    POND_MODULE(MotorTester),
    POND_MODULE(DummyMotor)
)