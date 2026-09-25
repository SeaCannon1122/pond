#include <pond/pond.hpp>

EXTERN_POND_MODULE(DummyCamera);
EXTERN_POND_MODULE(DepthColorizer);
EXTERN_POND_MODULE(MotorTestCmd);
EXTERN_POND_MODULE(MotorTestFeedback);
EXTERN_POND_MODULE(DummyMotor);
EXTERN_POND_MODULE(ChannelFilter);
EXTERN_POND_MODULE(Caller);
EXTERN_POND_MODULE(Receiver);
EXTERN_POND_MODULE(MotorControllerManager);

POND_BUNDLE_DECLARE(
    "Utility Modules",
    POND_MODULE(DummyCamera),
    POND_MODULE(DepthColorizer),
    POND_MODULE(MotorTestCmd),
    POND_MODULE(MotorTestFeedback),
    POND_MODULE(DummyMotor),
    POND_MODULE(ChannelFilter),
    POND_MODULE(Caller),
    POND_MODULE(Receiver),
    POND_MODULE(MotorControllerManager)
)