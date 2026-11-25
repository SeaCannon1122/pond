#include "pond/hpp/dds.hpp"
#include <pond/pond.hpp>

#include <pond_data_types/trajectory_types.hpp>
#include <vector>

class RobotTrajectoryPlanner : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
private:
    pond::Callback stage_callback;

    double dt;
    uint32_t step_count;
    std::vector<TrajectoryPoint> trajectory;
};

POND_MODULE_CPP_DECLARE(RobotTrajectoryPlanner, "robot_trajectory_planner", "plans the target trajectory the robot body should follow")

pond_result RobotTrajectoryPlanner::onStartup(const std::vector<void*>& args)
{
    dt = parameter("dt").asDouble().get(0.03);
    step_count = parameter("step_count").asInt().get(12);

    return POND_SUCCESS;
}

void RobotTrajectoryPlanner::onShutdown()
{
}