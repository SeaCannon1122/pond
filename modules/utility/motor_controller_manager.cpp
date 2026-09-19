#include <pond/pond.hpp>
#include <pond_data_types/motor_types.hpp>

class MotorControllerManager : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
    pond::Distributor command_distributor, feedback_distributor, update_distributor;
    std::vector<MotorInterface> interfaces;
    std::vector<MotorInterface*> interface_ptrs;
    std::vector<MotorCommand*> command_ptrs;
    std::vector<MotorFeedback*> feedback_ptrs;
};

POND_MODULE_CPP_DECLARE(MotorControllerManager, "motor_controller_manager", "connecting motors and controllers")

pond_result MotorControllerManager::onStartup(const std::vector<void*>& args)
{
    pond::ChannelsInfo command_info, feedback_info, update_info;

    auto motor_names = parameter("motor_names").asStringArray().getStrict(1, 0);
    if (!motor_names) return POND_ERROR;

    interfaces.resize(motor_names->size());
    interface_ptrs.resize(motor_names->size());
    command_ptrs.resize(motor_names->size());
    feedback_ptrs.resize(motor_names->size());

    for (uint32_t i = 0; i < motor_names->size(); i++)
    {
        interface_ptrs[i] = &interfaces[i];
        command_ptrs[i] = &interfaces[i].command;
        feedback_ptrs[i] = &interfaces[i].feedback;
    }

    for (auto& name : *motor_names)
    {
        command_info.channel<MotorCommand>(name + "/command");
        feedback_info.channel<MotorFeedback>(name + "/get_feedback");
        update_info.channel<MotorInterface>(name + "/update");
    }

    command_distributor = createDistributor(command_info);
    feedback_distributor = createDistributor(feedback_info);
    update_distributor = createDistributor(update_info);


    return POND_SUCCESS;
}

void MotorControllerManager::onShutdown()
{
    command_distributor.destroy();
    feedback_distributor.destroy();
    update_distributor.destroy();
}

void MotorControllerManager::onFrame()
{
    // reset interfaces;
    std::fill(interfaces.begin(), interfaces.end(), MotorInterface());

    // get feedback
    feedback_distributor.distribute(feedback_ptrs);

    //process through controllers
    update_distributor.distribute(interface_ptrs);
    

    // send motor commands
    command_distributor.distribute(command_ptrs);
}
