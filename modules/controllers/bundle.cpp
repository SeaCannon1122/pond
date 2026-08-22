#define POND_MODULE_CPP_MAKE_IMPLEMENTATION
#include <pond/pond.hpp>

EXTERN_POND_MODULE(DiffDriveController);

POND_BUNDLE_DECLARE(
    "Controllers for different hardware components", 
    1,
    POND_MODULE(DiffDriveController),
)