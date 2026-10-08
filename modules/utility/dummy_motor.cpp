#include "pond_data_types/motor_types.hpp"
#include <algorithm>
#include <cstdlib>
#include <pond/pond.hpp>
#include <pond_data_types/motor_types.hpp>

struct motor
{
    std::string name;

    double cmd_pos = 0;
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
    double max_speed = 0;
};

POND_MODULE_CPP_DECLARE(DummyMotor, "dummy_motor", "mock motor")

pond_result DummyMotor::onStartup(const std::vector<void*>& args)
{
    auto mode_o = parameter("mode").asString().getStrict({"position", "velocity"});
    if (!mode_o) return POND_ERROR;

    mode_pos = (*mode_o == "position");
    mode_vel = (*mode_o == "velocity");
    max_speed = std::abs(parameter("max_speed").asDouble().get(0.0));

    auto names = parameter("motor_names").asStringArray().get({"motor"});
    motors.reserve(names.size());

    pond::ChannelsInfo cmd_info, fb_info;
    for (auto& name : names)
    {
        motors.push_back({.name = name});
        cmd_info.channel<MotorCommand>(name + "/command");
        fb_info.channel<MotorFeedback>(name + "/get_feedback");
    }

    feedback_receiver = createReceiver<MotorFeedback>(fb_info, [this](MotorFeedback** feedbacks) {

        double time = pond::get_time();
        if (last_time == 0) last_time = time;
        double dt = time - last_time;

        for (uint32_t i = 0; i < motors.size(); i++)
        {
            if (mode_pos)
            {
                double cmd_speed = std::abs(motors[i].vel);
                double speed = (cmd_speed == 0 ? max_speed : cmd_speed);
                
                if (speed == 0)
                {
                    motors[i].pos = motors[i].cmd_pos;
                    feedbacks[i]->pos = motors[i].cmd_pos;
                    feedbacks[i]->vel = 0;
                }
                else
                {
                    double diff = motors[i].cmd_pos - motors[i].pos;
                    double abs_diff = std::abs(diff);

                    double delta = dt * speed;
                    if (delta < abs_diff)
                    {
                        double sign = (motors[i].cmd_pos > motors[i].pos ? 1.0 : -1.0);
                        motors[i].pos += sign * dt * speed;
                        feedbacks[i]->pos = motors[i].pos;
                        feedbacks[i]->vel = sign * speed;
                    }
                    else
                    {
                        motors[i].pos = motors[i].cmd_pos;
                        feedbacks[i]->pos = motors[i].cmd_pos;
                        feedbacks[i]->vel = 0;
                    }
                }
            }
            if (mode_vel)
            {
                motors[i].pos += motors[i].vel * dt;

                feedbacks[i]->pos = motors[i].pos;
                feedbacks[i]->vel = motors[i].vel;
            }
            feedbacks[i]->time = time;
            feedbacks[i]->hw_time = time;
        }
        last_time = time;
    });

    command_receiver = createReceiver<MotorCommand>(cmd_info, [this](MotorCommand** commands) {

        for (uint32_t i = 0; i < motors.size(); i++)
        {
            if (commands[i]->pos)
            {
                if(std::isfinite(*commands[i]->pos)) motors[i].cmd_pos = *commands[i]->pos;
                else POND_LOG("Cannot set motor '%s' to position %f", motors[i].name.c_str(), *commands[i]->pos);
            }
            if (commands[i]->vel)
            {
                if(std::isfinite(*commands[i]->vel)) motors[i].vel = *commands[i]->vel;
                else POND_LOG("Cannot set motor '%s' to velocity %f", motors[i].name.c_str(), *commands[i]->vel);
            }
        }
    });

    return POND_SUCCESS;
}

void DummyMotor::onShutdown()
{
    feedback_receiver.destroy();
    command_receiver.destroy();
}
