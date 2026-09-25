#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>

EXTERN_POND_MODULE(DiffDriveController);
EXTERN_POND_MODULE(ArmController);
EXTERN_POND_MODULE(AngleGripperController);

POND_BUNDLE_DECLARE(
    "Controllers for different hardware components",
    POND_MODULE(DiffDriveController),
    POND_MODULE(ArmController),
    POND_MODULE(AngleGripperController)
)