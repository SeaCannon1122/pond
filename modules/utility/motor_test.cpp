#include "pond/pond.h"
#include <pond/pond.hpp>
#include <pond/data_types/motor_types.hpp>

class MotorTestCmd : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
private:
};

POND_MODULE_CPP_DECLARE(MotorTestCmd, "motor_test_cmd", "module for sending test motor command")

pond_result MotorTestCmd::onStartup(const std::vector<void*>& args)
{
    std::vector<MotorCommand> cmds(1);
    cmds[0].vel = parameter("velocity").asDouble().getStrict({}, false);
    cmds[0].pos = parameter("position").asDouble().getStrict({}, false);
    cmds[0].torque = parameter("torque").asDouble().getStrict({}, false);
    cmds[0].disable = parameter("disable").asBool().getStrict({}, false);

    pond::Distributor<std::vector<MotorCommand>> command_distributor = createDistributor<std::vector<MotorCommand>>({"motor_cmd"});
    command_distributor.distribute(cmds);

    command_distributor.destroy();
    shutdown();
    return POND_SUCCESS;
}

class MotorTestFeedback : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
private:
};

POND_MODULE_CPP_DECLARE(MotorTestFeedback, "motor_test_feedback", "module for receiving feedback from a motor")

pond_result MotorTestFeedback::onStartup(const std::vector<void*>& args)
{
    std::vector<MotorFeedback> feedback(1);

    pond::Distributor<std::vector<MotorFeedback>> feedback_distributor = createDistributor<std::vector<MotorFeedback>>({"get_motor_feedback"});
    feedback_distributor.distribute(feedback);

    POND_LOG(
        "Feedback:\n"
        "  pos: %s\n"
        "  vel: %s\n"
        "  torque: %s\n"
        "  current: %s\n"
        "  temperature: %s\n",
        feedback[0].pos ? std::to_string(*feedback[0].pos).c_str() : "--",
        feedback[0].vel ? std::to_string(*feedback[0].vel).c_str() : "--",
        feedback[0].torque ? std::to_string(*feedback[0].torque).c_str() : "--",
        feedback[0].current ? std::to_string(*feedback[0].current).c_str() : "--",
        feedback[0].temperature ? std::to_string(*feedback[0].temperature).c_str() : "--"
    );

    feedback_distributor.destroy();
    shutdown();
    return POND_SUCCESS;
}