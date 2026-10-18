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

    MotorFeedback feedback;
};

class DummyMotor : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
private:
    pond::Callback command_callback, feedback_callback;
    std::string feedback_stage_name, command_stage_name;

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
    feedback_stage_name = parameter("feedback_stage_name").asString().get("read_motor_feedback");
    command_stage_name = parameter("command_stage_name").asString().get("write_motor_commands");
    
    auto names = parameter("motor_names").asStringArray().get({"motor"});
    motors.reserve(names.size());

    pond::ChannelsInfo cmd_info, fb_info;
    for (auto& name : names)
    {
        motors.push_back({.name = name});
        cmd_info.channel<MotorCommand*>(command_stage_name+"/" + name + "_command");
        fb_info.channel<MotorFeedback*>(feedback_stage_name+"/" + name + "_feedback");
    }

    feedback_callback = createCallback<MotorFeedback*>(fb_info, [this](MotorFeedback*** feedbacks) {

        for (uint32_t i = 0; i < motors.size(); i++) *feedbacks[i] = &motors[i].feedback;

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
                    motors[i].feedback.pos = motors[i].cmd_pos;
                    motors[i].feedback.vel = 0;
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
                        motors[i].feedback.pos = motors[i].pos;
                        motors[i].feedback.vel = sign * speed;
                    }
                    else
                    {
                        motors[i].pos = motors[i].cmd_pos;
                        motors[i].feedback.pos = motors[i].cmd_pos;
                        motors[i].feedback.vel = 0;
                    }
                }
            }
            if (mode_vel)
            {
                motors[i].pos += motors[i].vel * dt;

                motors[i].feedback.pos = motors[i].pos;
                motors[i].feedback.vel = motors[i].vel;
            }
            motors[i].feedback.time = time;
            motors[i].feedback.hw_time = time;
        }
        last_time = time;
    });

    command_callback = createCallback<MotorCommand*>(cmd_info, [this](MotorCommand*** commands) {

        for (uint32_t i = 0; i < motors.size(); i++) if (*commands[i] == NULL) POND_LOG_RETURN("%s is NULL", (command_stage_name+"/" + motors[i].name + "_command").c_str());

        for (uint32_t i = 0; i < motors.size(); i++)
        {
            if (commands[i][0]->pos)
            {
                if(std::isfinite(*commands[i][0]->pos)) motors[i].cmd_pos = *commands[i][0]->pos;
                else POND_LOG("Cannot set motor '%s' to position %f", motors[i].name.c_str(), *commands[i][0]->pos);
            }
            if (commands[i][0]->vel)
            {
                if(std::isfinite(*commands[i][0]->vel)) motors[i].vel = *commands[i][0]->vel;
                else POND_LOG("Cannot set motor '%s' to velocity %f", motors[i].name.c_str(), *commands[i][0]->vel);
            }
        }
    });

    return POND_SUCCESS;
}

void DummyMotor::onShutdown()
{
    feedback_callback.destroy();
    command_callback.destroy();
}
