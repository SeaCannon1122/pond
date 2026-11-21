#include <pond/pond.hpp>

EXTERN_POND_MODULE(GaitScheduler);
EXTERN_POND_MODULE(SwingTrajectoryController);
EXTERN_POND_MODULE(LegKinematicsController);
EXTERN_POND_MODULE(MotorAbstractor);
EXTERN_POND_MODULE(DebugDistributor);
EXTERN_POND_MODULE(LegTorqueController);
EXTERN_POND_MODULE(MPCController);
EXTERN_POND_MODULE(CommandController);
EXTERN_POND_MODULE(FootContactController);
EXTERN_POND_MODULE(RobotTrajectoryPlanner);

POND_BUNDLE_DECLARE(
    "Controllers for a quadruped robodog",
    POND_MODULE(GaitScheduler),
    POND_MODULE(SwingTrajectoryController),
    POND_MODULE(LegKinematicsController),
    POND_MODULE(MotorAbstractor),
    POND_MODULE(DebugDistributor),
    POND_MODULE(LegTorqueController),
    POND_MODULE(MPCController),
    POND_MODULE(CommandController),
    POND_MODULE(FootContactController),
    POND_MODULE(RobotTrajectoryPlanner)
)