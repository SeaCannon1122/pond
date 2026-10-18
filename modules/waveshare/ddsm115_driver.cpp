#include <pond/pond.hpp>
#include <pond_data_types/motor_types.hpp>
#include "DDSM115CMD.h"
#include "pond/hpp/module_base.hpp"

struct ddsm115_motor
{
    double last_position = 0;
    double cmd_velocity = 0;
    
    double feedback_pos = 0;
    double feedback_vel = 0;

    int32_t id;
    double scalar;
    bool read = false;

    std::string name;
    MotorFeedback feedback;
};

class DDSM115Driver : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
private:
    std::vector<ddsm115_motor> motors;
    DDSM115CMD cmd;

    int32_t act;

    double last_time = 0;
    pond::Callback command_callback, feedback_callback;
    std::string feedback_stage_name, command_stage_name;
};

POND_MODULE_CPP_DECLARE(DDSM115Driver, "ddsm115_driver", "driver for the DDSM115 Motors")

pond_result DDSM115Driver::onStartup(const std::vector<void*>& args)
{
    feedback_stage_name = parameter("feedback_stage_name").asString().get("read_motor_feedback");
    command_stage_name = parameter("command_stage_name").asString().get("write_motor_commands");

    act = parameter("act").asInt().get(3);
    auto device = parameter("device").asString().getStrict();
    if (!device) return POND_ERROR;

    auto space = parameterSpace("motors");
    if (uint32_t motor_count = space.listLength("name"); motor_count != 0) motors.resize(motor_count);
    else POND_LOG_RETURN_ERROR("Did not find motor declaration");

    pond::ChannelsInfo cmd_info, fb_info;
    for (uint32_t i = 0; i < motors.size(); i++)
    {
        motors[i].name = space.parameterAtIndex(i, "name").asString().get("motor_name");
        motors[i].id = space.parameterAtIndex(i, "id").asInt().get(255);
        motors[i].scalar = (space.parameterAtIndex(i, "invert").asBool().get(false) ? -1 : 1);
    
        cmd_info.channel<MotorCommand*>(command_stage_name + "/" + motors[i].name + "_command");
        fb_info.channel<MotorFeedback*>(feedback_stage_name + "/" + motors[i].name + "_feedback");
    }
    
    if (cmd.connect(*device) == false) POND_LOG_RETURN_ERROR(cmd.get_error());

    feedback_callback = createCallback<MotorFeedback*>(fb_info, [this](MotorFeedback*** feedbacks) {

        for (uint32_t i = 0; i < motors.size(); i++) *feedbacks[i] = &motors[i].feedback;

        double current_time = pond::get_time();
        double dt = current_time - last_time;
        last_time = current_time;

        for (size_t i = 0; i < motors.size(); i++)
        {
            double vel = motors[i].cmd_velocity, pos = motors[i].feedback_pos + motors[i].feedback_vel * dt, cur = 0.;

            uint8_t fb_id, fb_mode, fb_error_code;
            double fb_vel, fb_pos, fb_cur;

            if (cmd.drive_feedback(&fb_id, &fb_mode, &fb_pos, &fb_vel, &fb_cur, &fb_error_code) == false) POND_LOG(cmd.get_error());
            else if (fb_id != motors[i].id) POND_LOG("Received response for wheel %d instead of %d", fb_id, motors[i].id);
            else
            {
                if (motors[i].read == false)
                {
                    motors[i].last_position = fb_pos;
                    motors[i].read = true;
                }

                vel = (double)motors[i].scalar * fb_vel;

                double delta = fb_pos - motors[i].last_position;
                motors[i].last_position = fb_pos;

                if (delta > M_PI) delta -= 2.0 * M_PI;
                if (delta < -M_PI) delta += 2.0 * M_PI;

                pos = motors[i].feedback_pos  - (double)motors[i].scalar * delta;
                cur = fb_cur;
            }

            motors[i].feedback.vel = vel;
            motors[i].feedback.pos = pos;
            motors[i].feedback.current = cur;
            motors[i].feedback.time = current_time;
            motors[i].feedback.hw_time = current_time;

            motors[i].feedback_pos = pos;
            motors[i].feedback_vel = vel;
        }
    });

    command_callback = createCallback<MotorCommand*>(cmd_info, [this](MotorCommand*** commands) {

        for (uint32_t i = 0; i < motors.size(); i++) if (*commands[i] == NULL)
            POND_LOG_RETURN("%s is NULL", (command_stage_name + "/" + motors[i].name + "_command").c_str());


        for (size_t i = 0; i < motors.size(); i++)
        {
            motors[i].cmd_velocity = (commands[i][0]->vel ? *commands[i][0]->vel : 0);

            if (cmd.drive(motors[i].id, motors[i].cmd_velocity * motors[i].scalar, act, 0) == false) POND_LOG(cmd.get_error());
        }
    });

    return POND_SUCCESS;
}

void DDSM115Driver::onShutdown()
{
    feedback_callback.destroy();
    command_callback.destroy();
    cmd.disconnect();
}
