#include <pond/pond.hpp>

class TrajectoryPlanner : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
private:
};

POND_MODULE_CPP_DECLARE(TrajectoryPlanner, "trajectory_planner", "calculating and evaluating the feet trajectories")

pond_result TrajectoryPlanner::onStartup(const std::vector<void*>& args)
{
    return POND_SUCCESS;
}

void TrajectoryPlanner::onShutdown()
{
}
