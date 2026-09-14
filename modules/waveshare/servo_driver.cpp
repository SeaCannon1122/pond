#include <pond/pond.hpp>
#include <pond/data_types/motor_types.hpp>
#include <cmath>

#include "pond/pond.h"
#include "sms_sts/SMS_STS.h"

#define KT 9.0 // torque constant (kg*cm / A)
#define STEPS 4096.0
#define MAX_SPEED 6000 // 6000;
#define MAX_ACC 150 // 150;

struct servo
{
    u8 id;
    double scalar;
    double offset;
    double pos_min;
    double pos_max;
};

class ServoDriver : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;

private:
    pond::Receiver<std::vector<MotorCommand>> command_receiver;
    pond::Receiver<std::vector<MotorFeedback>> feedback_receiver;

    std::vector<servo> servos;
    std::vector<u8> ids;
    std::vector<s16> positions;
    std::vector<u16> speeds;
    std::vector<u8> accelerations;
    SMS_STS sm_st;
};

POND_MODULE_CPP_DECLARE(ServoDriver, "servo_driver", "servo driver module")

pond_result ServoDriver::onStartup(const std::vector<void*>& args)
{
    auto device = parameter("device").asString().getStrict();
    auto baudrate = parameter("baudrate").asInt().getStrict();
    if (!device || !baudrate) return POND_ERROR;

    uint32_t servo_count = 0;
    while (parameter("servo" + std::to_string(servo_count) + ".id").asInt().getStrict({}, false)) servo_count++;

    if (servo_count == 0) {POND_LOG("Error: Did not find any servo definitions"); return POND_ERROR;}

    servos.resize(servo_count);
    ids.resize(servo_count);
    positions.resize(servo_count);
    speeds.resize(servo_count);
    accelerations.resize(servo_count);

    for (uint32_t i = 0; i < servos.size(); i++)
    {
        std::string prefix = "servo" + std::to_string(i);
        
        auto id = parameter(prefix+".id").asInt().getStrict();
        auto min = parameter(prefix+".pos_min").asDouble().getStrict();
        auto max = parameter(prefix+".pos_max").asDouble().getStrict();

        if (!id || !min || !max) return POND_ERROR;
        
        servos[i].id = *id;
        servos[i].pos_min = *min;
        servos[i].pos_max = *max;

        servos[i].offset = parameter(prefix+".offset").asDouble().get(0);
        servos[i].scalar = (parameter(prefix + ".invert").asBool().get(false) ? -1 : 1);
    }

    if (!sm_st.begin(*baudrate, device->c_str()))
    {
        POND_LOG("Error: Failed do start on port %s", device->c_str());
        return POND_ERROR;
    }

    for (size_t i = 0; i < servos.size(); i++)
    {
        if (sm_st.Ping(servos[i].id) == -1)
        {
            POND_LOG("Error: unable to ping motor id '%d'", servos[i].id);
            sm_st.end();
            return POND_ERROR;
        }
        else sm_st.Mode(servos[i].id, 0);
    }

    feedback_receiver = createReceiver<std::vector<MotorFeedback>>({"get_motor_feedback"}, [this](std::vector<MotorFeedback>* feedbacks) {
        if (feedbacks->size() < servos.size()) { POND_LOG("feedbacks size (%d) < servos size (%d)", feedbacks->size(), servos.size()); return; }

        for (size_t i = 0; i < servos.size(); i++)
        {
            feedbacks->at(i).pos = servos[i].scalar * ((double)sm_st.ReadPos(servos[i].id) * 2 * M_PI / STEPS - servos[i].offset);
            feedbacks->at(i).vel = servos[i].scalar * (double)sm_st.ReadSpeed(servos[i].id) * 2 * M_PI / STEPS;
            // ReadCurrent(ID) return unitless value, multiply by static current (6mA)
            feedbacks->at(i).current = (double)sm_st.ReadCurrent(servos[i].id) * 6.0 / 1000.0;
            feedbacks->at(i).torque = servos[i].scalar * *feedbacks->at(i).current * KT;
            feedbacks->at(i).temperature = sm_st.ReadTemper(servos[i].id);
        }

    });

    command_receiver = createReceiver<std::vector<MotorCommand>>({"motor_cmd"}, [this](std::vector<MotorCommand>* commands) {

        if (commands->size() < servos.size()) { POND_LOG("commands size (%d) < servos size (%d)", commands->size(), servos.size()); return; }

        uint32_t cmd_count = 0;

        for (size_t i = 0; i < servos.size(); i++)
        {
            if (commands->at(i).disable) sm_st.EnableTorque((u8)servos[i].id, *commands->at(i).disable ? 0 : 1);
            else if (commands->at(i).pos)
            {
                double cmd_pos = *commands->at(i).pos;
                if (cmd_pos < servos[i].pos_min) cmd_pos = servos[i].pos_min;
                if (cmd_pos > servos[i].pos_max) cmd_pos = servos[i].pos_max;

                double pos = servos[i].scalar * cmd_pos + servos[i].offset;
                positions[cmd_count] = static_cast<s16>((pos * STEPS) / (2 * M_PI));

                if (commands->at(i).vel) speeds[cmd_count] = static_cast<u16>(std::clamp((std::abs(*commands->at(i).vel) * STEPS) / (2 * M_PI), -32767.0, 32767.0));
                else speeds[cmd_count] = 0;

                if (commands->at(i).acc) accelerations[cmd_count] = static_cast<u16>(std::clamp(*commands->at(i).acc * 41.0 / (2 * M_PI), 0.0, 254.0));
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

    feedback_receiver.destroy();
    command_receiver.destroy();
    sm_st.end();
}