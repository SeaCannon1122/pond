#include <pond/pond.hpp>
#include <pond_data_types/motor_types.hpp>

class MotorTestCmd : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
private:
};

POND_MODULE_CPP_DECLARE(MotorTestCmd, "motor_test_cmd", "module for sending test motor command")

pond_result MotorTestCmd::onStartup(const std::vector<void*>& args)
{
    MotorCommand cmd;
    cmd.vel = parameter("velocity").asDouble().getStrict({}, false);
    cmd.pos = parameter("position").asDouble().getStrict({}, false);
    cmd.torque = parameter("torque").asDouble().getStrict({}, false);
    cmd.disable = parameter("disable").asBool().getStrict({}, false);
    auto motor_name = parameter("motor_name").asString().get("motor");

    pond::Distributor distributor = createDistributorTyped<MotorCommand>({motor_name +"/command"});
    distributor.distribute(&cmd);
    distributor.destroy();
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
    MotorFeedback feedback;
    auto motor_name = parameter("motor_name").asString().get("motor");

    pond::Distributor distributor = createDistributor<MotorFeedback>({motor_name + "/get_feedback"});
    distributor.distribute(&feedback);

    POND_LOG(
        "Feedback:\n"
        "  pos: %s\n"
        "  vel: %s\n"
        "  torque: %s\n"
        "  current: %s\n"
        "  temperature: %s\n",
        feedback.pos ? std::to_string(*feedback.pos).c_str() : "--",
        feedback.vel ? std::to_string(*feedback.vel).c_str() : "--",
        feedback.torque ? std::to_string(*feedback.torque).c_str() : "--",
        feedback.current ? std::to_string(*feedback.current).c_str() : "--",
        feedback.temperature ? std::to_string(*feedback.temperature).c_str() : "--"
    );

    distributor.destroy();
    shutdown();
    return POND_SUCCESS;
}