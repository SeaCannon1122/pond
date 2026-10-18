#include <pond/pond.hpp>
#include <pond_data_types/motor_types.hpp>
#include <cmath>

#include "pond/hpp/module_base.hpp"
#include "sms_sts/SMS_STS.h"

#define KT 9.0 // torque constant (kg*cm / A)
#define STEPS 4096.0
#define MAX_SPEED 6000 // 6000;
#define MAX_ACC 150 // 150;

struct servo
{
    std::string name;

    u8 id;
    double scalar;
    double offset;
    double pos_min;
    double pos_max;

    MotorFeedback feedback;
};

class ServoDriver : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;

private:
    pond::Callback command_callback, feedback_callback;

    std::vector<servo> servos;
    std::vector<u8> ids;
    std::vector<s16> positions;
    std::vector<u16> speeds;
    std::vector<u8> accelerations;
    SMS_STS sm_st;

    std::string feedback_stage_name, command_stage_name;
};

POND_MODULE_CPP_DECLARE(ServoDriver, "servo_driver", "servo driver module")

pond_result ServoDriver::onStartup(const std::vector<void*>& args)
{
    feedback_stage_name = parameter("feedback_stage_name").asString().get("read_servo_feedback");
    command_stage_name = parameter("command_stage_name").asString().get("write_servo_commands");

    auto device = parameter("device").asString().getStrict();
    auto baudrate = parameter("baudrate").asInt().getStrict();
    if (!device || !baudrate) return POND_ERROR;

    if (!sm_st.begin(*baudrate, device->c_str())) POND_LOG_RETURN_ERROR("Error: Failed do start on port %s", device->c_str());

    auto servo_params = parameterSpace("servos");
    if (auto servo_count = servo_params.listLength("id"); servo_count != 0) servos.resize(servo_count);
    else POND_LOG_RETURN_ERROR("Error: Did not find any servo definitions"); 

    ids.resize(servos.size());
    positions.resize(servos.size());
    speeds.resize(servos.size());
    accelerations.resize(servos.size());
    pond::ChannelsInfo cmd_info, fb_info;

    for (uint32_t i = 0; i < servos.size(); i++)
    {
        auto min = servo_params.parameterAtIndex(i,"pos_min").asDouble().getStrict();
        auto max = servo_params.parameterAtIndex(i,"pos_max").asDouble().getStrict();
        auto name = servo_params.parameterAtIndex(i,"name").asString().getStrict();
        if (!min || !max || !name) { sm_st.end(); return POND_ERROR; }
        
        servos[i].name = *name;
        servos[i].id = servo_params.parameterAtIndex(i,"id").asInt().get(255);
        servos[i].pos_min = *min;
        servos[i].pos_max = *max;
        servos[i].offset = servo_params.parameterAtIndex(i,"offset").asDouble().get(0);
        servos[i].scalar = (servo_params.parameterAtIndex(i,"invert").asBool().get(false) ? -1 : 1);

        cmd_info.channel<MotorCommand*>(command_stage_name + "/" + *name + "_command");
        fb_info.channel<MotorFeedback*>(feedback_stage_name + "/" + *name + "_feedback");

        if (sm_st.Ping(servos[i].id) == -1)
        {
            sm_st.end();
            POND_LOG_RETURN_ERROR("Error: unable to ping motor id '%d'", servos[i].id);
        }
        else sm_st.Mode(servos[i].id, 0);
    }

    feedback_callback = createCallback<MotorFeedback*>(fb_info, [this](MotorFeedback*** feedbacks) {

        double time = pond::get_time();

        for (size_t i = 0; i < servos.size(); i++)
        {
            for (uint32_t i = 0; i < servos.size(); i++) *feedbacks[i] = &servos[i].feedback;

            servos[i].feedback.pos = servos[i].scalar * ((double)sm_st.ReadPos(servos[i].id) * 2 * M_PI / STEPS - servos[i].offset);
            servos[i].feedback.vel = servos[i].scalar * (double)sm_st.ReadSpeed(servos[i].id) * 2 * M_PI / STEPS;
            // ReadCurrent(ID) return unitless value, multiply by static current (6mA)
            servos[i].feedback.current = (double)sm_st.ReadCurrent(servos[i].id) * 6.0 / 1000.0;
            servos[i].feedback.torque = servos[i].scalar * *servos[i].feedback.current * KT;
            servos[i].feedback.temperature = sm_st.ReadTemper(servos[i].id);

            servos[i].feedback.time = time;
            servos[i].feedback.hw_time = time;
        }
    });

    command_callback = createCallback<MotorCommand*>(cmd_info, [this](MotorCommand*** commands) {

        for (uint32_t i = 0; i < servos.size(); i++) if (*commands[i] == NULL)
            POND_LOG_RETURN("%s is NULL", (command_stage_name + "/" + servos[i].name + "_command").c_str());

        uint32_t cmd_count = 0;

        for (size_t i = 0; i < servos.size(); i++)
        {
            if (commands[i][0]->disable) sm_st.EnableTorque((u8)servos[i].id, *commands[i][0]->disable ? 0 : 1);
            else if (commands[i][0]->pos)
            {
                double cmd_pos = *commands[i][0]->pos;
                if (cmd_pos < servos[i].pos_min) cmd_pos = servos[i].pos_min;
                if (cmd_pos > servos[i].pos_max) cmd_pos = servos[i].pos_max;

                double pos = servos[i].scalar * cmd_pos + servos[i].offset;
                positions[cmd_count] = static_cast<s16>((pos * STEPS) / (2 * M_PI));

                if (commands[i][0]->vel) speeds[cmd_count] = static_cast<u16>(std::clamp((std::abs(*commands[i][0]->vel) * STEPS) / (2 * M_PI), -32767.0, 32767.0));
                else speeds[cmd_count] = 0;

                if (commands[i][0]->acc) accelerations[cmd_count] = static_cast<u16>(std::clamp(*commands[i][0]->acc * 41.0 / (2 * M_PI), 0.0, 254.0));
                else accelerations[cmd_count] = MAX_ACC;

                ids[cmd_count] = servos[i].id;

                cmd_count++;
            }
        }
        sm_st.SyncWritePosEx(ids.data(), static_cast<u8>(cmd_count), positions.data(), speeds.data(), accelerations.data());
    });

    return POND_SUCCESS;
}

void ServoDriver::onShutdown()
{
    for (uint32_t i = 0; i < servos.size(); i++) sm_st.EnableTorque((u8)servos[i].id, 0);

    feedback_callback.destroy();
    command_callback.destroy();
    sm_st.end();
}