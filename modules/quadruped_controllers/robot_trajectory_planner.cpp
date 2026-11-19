#include <pond/pond.hpp>

class RobotTrajectoryPlanner : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
private:
};

POND_MODULE_CPP_DECLARE(RobotTrajectoryPlanner, "robot_trajectory_planner", "plans the target trajectory the robot body should follow")

pond_result RobotTrajectoryPlanner::onStartup(const std::vector<void*>& args)
{
    return POND_SUCCESS;
}

void RobotTrajectoryPlanner::onShutdown()
{
}