#define POND_MODULE_CPP_MAKE_IMPLEMENTATION
#include <pond/module_base_tf.hpp>

EXTERN_POND_MODULE(DiffDriveController);
EXTERN_POND_MODULE(ArmController);

POND_BUNDLE_DECLARE(
    "Controllers for different hardware components",
    POND_MODULE(DiffDriveController),
    POND_MODULE(ArmController)
)