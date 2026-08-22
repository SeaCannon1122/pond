#include <pond/pond.hpp>
#include <pond/data_types/motor_types.hpp>
#include "DDSM115CMD.h"
#include <chrono>

struct ddsm115_motor
{
  double last_position = 0;
  double cmd_velocity = 0;

  int32_t id;
  double scalar;
  bool read = false;
};

class DDSM115Driver : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
    std::vector<ddsm115_motor> motors;
    std::vector<MotorFeedback> feedback;
    DDSM115CMD cmd;

    int32_t act;

    double last_time = 0;
    pond::Receiver<std::vector<MotorCommand>> receiver;
    pond::Distributor<std::vector<MotorFeedback>> distributor;
};

POND_MODULE_CPP_DECLARE(DDSM115Driver, "ddsm115_driver", "driver for the DDSM115 Motors")

pond_result DDSM115Driver::onStartup(const std::vector<void*>& args)
{
    auto device = parameter("device").asString().getStrict();
    auto motor_count = parameter("motor_count").asInt().getStrict();
    if (!device || !motor_count) return POND_ERROR;

    if (*motor_count < 1){
        POND_LOG("ERROR: parameter 'motor_count' (%d) must be at least 1", *motor_count);
        return POND_ERROR;
    }

    motors.resize(*motor_count);
    feedback.resize(*motor_count);

    if ((act = parameter("act").asInt().get(3)) < 1) {
        POND_LOG("ERROR: parameter 'act' (%d) must be at least 1", act);
        return POND_ERROR;
    }

    for (uint32_t i = 0; i < motors.size(); i++)
    {
        std::string id_param = "motor" + std::to_string(i) + ".id";
        
        if (auto id = parameter(id_param).asInt().getStrict())
        {
            if ((motors[i].id = *id) < 1) {
                POND_LOG("ERROR: parameter '%s' (%d) must be at least 1", id_param.c_str(), *id);
                return POND_ERROR;
            }
        }
        else return POND_ERROR;

        motors[i].scalar = (parameter("motor" + std::to_string(i) + ".invert").asBool().get(false) ? -1 : 1);
    }
    
    if (cmd.connect(*device) == false)
    {
        POND_LOG(cmd.get_error());
        return POND_ERROR;
    }

    distributor = createDistributor<std::vector<MotorFeedback>>({"motor_feedback"});
    receiver = createReceiver<std::vector<MotorCommand>>({"motor_cmd"}, [this](std::vector<MotorCommand>* commands) {
        
        if (commands->size() != motors.size())
        {
            POND_LOG("ERROR: commands->size (%s) != motors.size (%s)", commands->size(), motors.size());
            return;
        }

        double current_time = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        double dt = current_time - last_time;
        last_time = current_time;

        for (size_t i = 0; i < motors.size(); i++)
        {
            double vel = motors[i].cmd_velocity, pos = feedback[i].pos + feedback[i].vel * dt, cur = 0.;

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
                else if (delta < -M_PI) delta += 2.0 * M_PI;

                pos = feedback[i].vel - (double)motors[i].scalar * delta;
                cur = fb_cur;
            }

            feedback[i].vel = vel;
            feedback[i].pos = pos;
            feedback[i].current = cur;
        }
        
        for (size_t i = 0; i < motors.size(); i++)
        {
            motors[i].cmd_velocity = (*commands)[i].vel;

            if (cmd.drive(motors[i].id, motors[i].cmd_velocity  * motors[i].scalar, act, 0) == false)
                POND_LOG(cmd.get_error());
        }

        distributor.distribute(feedback); 
    });

    return POND_SUCCESS;
}

void DDSM115Driver::onShutdown()
{
    receiver.destroy();
    distributor.destroy();
    cmd.disconnect();
}

void DDSM115Driver::onFrame()
{
}
