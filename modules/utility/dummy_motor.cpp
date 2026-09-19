#include "pond_data_types/motor_types.hpp"
#include <pond/pond.hpp>
#include <pond_data_types/motor_types.hpp>

struct motor
{
    double pos = 0;
    double vel = 0;
};

class DummyMotor : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
private:
    pond::Receiver command_receiver;
    pond::Receiver feedback_receiver;

    bool mode_pos;
    bool mode_vel;
    std::vector<motor> motors;
    double last_time = 0;
};

POND_MODULE_CPP_DECLARE(DummyMotor, "dummy_motor", "mock motor")

pond_result DummyMotor::onStartup(const std::vector<void*>& args)
{
    auto mode_o = parameter("mode").asString().getStrict({"position", "velocity"});
    if (!mode_o) return POND_ERROR;

    mode_pos = (*mode_o == "position");
    mode_vel = (*mode_o == "velocity");

    auto names = parameter("motor_names").asStringArray().get({"motor"});
    motors.resize(names.size());

    pond::ChannelsInfo cmd_info, fb_info;
    for (auto& name : names)
    {
        cmd_info.channel<MotorCommand>(name + "/command");
        fb_info.channel<MotorFeedback>(name + "/get_feedback");
    }

    feedback_receiver = createReceiver<MotorFeedback>(fb_info, [this](MotorFeedback** feedbacks) {

        double time = pond::get_time();
        if (last_time == 0) last_time = time;

        for (uint32_t i = 0; i < motors.size(); i++)
        {
            if (mode_pos)
            {
                feedbacks[i]->pos = motors[i].pos;
                feedbacks[i]->vel = 0;
            }
            if (mode_vel)
            {
                motors[i].pos += motors[i].vel * (time - last_time);

                feedbacks[i]->pos = motors[i].pos;
                feedbacks[i]->vel = motors[i].vel;
            }
            feedbacks[i]->time = time;
            feedbacks[i]->hw_time = time;
        }
        last_time = time;
    });

    command_receiver = createReceiver<MotorCommand>(cmd_info, [this](MotorCommand** commands) {

        if (last_time == 0) last_time = pond::get_time();
        for (uint32_t i = 0; i < motors.size(); i++)
        {
            if (commands[i]->pos) motors[i].pos = *commands[i]->pos;
            if (commands[i]->vel) motors[i].vel = *commands[i]->vel;
        }
    });

    return POND_SUCCESS;
}

void DummyMotor::onShutdown()
{
    feedback_receiver.destroy();
    command_receiver.destroy();
}
