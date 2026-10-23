#include "pond/h/module.h"
#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>

EXTERN_POND_MODULE(GaitScheduler);
EXTERN_POND_MODULE(TrajectoryPlanner);
EXTERN_POND_MODULE(LegKinematicsController);
EXTERN_POND_MODULE(MotorJointUpdater);
EXTERN_POND_MODULE(LegTorqueController);
EXTERN_POND_MODULE(MPCController);

POND_BUNDLE_DECLARE(
    "Controllers for a quadruped robodog",
    POND_MODULE(GaitScheduler),
    POND_MODULE(TrajectoryPlanner),
    POND_MODULE(LegKinematicsController),
    POND_MODULE(MotorJointUpdater),
    POND_MODULE(LegTorqueController),
    POND_MODULE(MPCController)
)