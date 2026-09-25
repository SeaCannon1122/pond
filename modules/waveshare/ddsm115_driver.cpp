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
    pond::Receiver command_receiver, feedback_receiver;
};

POND_MODULE_CPP_DECLARE(DDSM115Driver, "ddsm115_driver", "driver for the DDSM115 Motors")

pond_result DDSM115Driver::onStartup(const std::vector<void*>& args)
{
    act = parameter("act").asInt().get(3);
    auto device = parameter("device").asString().getStrict();
    if (!device) return POND_ERROR;

    auto space = parameterSpace("motors");
    if (uint32_t motor_count = space.listLength("name"); motor_count != 0) motors.resize(motor_count);
    else POND_LOG_RETURN_ERROR("Did not find motor declaration");

    pond::ChannelsInfo cmd_info, fb_info;
    for (uint32_t i = 0; i < motors.size(); i++)
    {
        auto name = space.parameterAtIndex(i, "name").asString().get("motor_name");
        motors[i].id = space.parameterAtIndex(i, "id").asInt().get(255);
        motors[i].scalar = (space.parameterAtIndex(i, "invert").asBool().get(false) ? -1 : 1);
    
        cmd_info.channel<MotorCommand>(name + "/command");
        fb_info.channel<MotorFeedback>(name + "/get_feedback");
    }
    
    if (cmd.connect(*device) == false) POND_LOG_RETURN_ERROR(cmd.get_error());

    feedback_receiver = createReceiver<MotorFeedback>(fb_info, [this](MotorFeedback** feedbacks) {

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

            feedbacks[i]->vel = vel;
            feedbacks[i]->pos = pos;
            feedbacks[i]->current = cur;
            feedbacks[i]->time = current_time;
            feedbacks[i]->hw_time = current_time;

            motors[i].feedback_pos = pos;
            motors[i].feedback_vel = vel;
        }
    });

    command_receiver = createReceiver<MotorCommand>(cmd_info, [this](MotorCommand** commands) {

        for (size_t i = 0; i < motors.size(); i++)
        {
            motors[i].cmd_velocity = (commands[i]->vel ? *commands[i]->vel : 0);

            if (cmd.drive(motors[i].id, motors[i].cmd_velocity * motors[i].scalar, act, 0) == false) POND_LOG(cmd.get_error());
        }
    });

    return POND_SUCCESS;
}

void DDSM115Driver::onShutdown()
{
    feedback_receiver.destroy();
    command_receiver.destroy();
    cmd.disconnect();
}
