#include <pond/pond.hpp>

class LegKinematicsController : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
private:
};

POND_MODULE_CPP_DECLARE(LegKinematicsController, "leg_kinematics_controller", "calculating the target leg angles")

pond_result LegKinematicsController::onStartup(const std::vector<void*>& args)
{
    return POND_SUCCESS;
}

void LegKinematicsController::onShutdown()
{
}
