#include "pond/pond.h"
#include <pond/pond.hpp>
#include <pond/data_types/motor_types.hpp>

class MotorTester : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
};

POND_MODULE_CPP_DECLARE(MotorTester, "motor_tester", "module for testing motor")

pond_result MotorTester::onStartup(const std::vector<void*>& args)
{
    std::vector<MotorCommand> cmds(1);
    auto vel = parameter("velocity").asDouble().getStrict();
    auto pos = parameter("position").asDouble().getStrict();
    if (!vel || !pos) return POND_ERROR;

    pond::Distributor<std::vector<MotorCommand>> command_distributor = createDistributor<std::vector<MotorCommand>>({"motor_cmd"});

    cmds[0].vel = *vel;
    cmds[0].pos = *pos;
    command_distributor.distribute(cmds);

    command_distributor.destroy();
    shutdown();
    return POND_SUCCESS;
}

void MotorTester::onShutdown()
{
}

void MotorTester::onFrame()
{
    
}
