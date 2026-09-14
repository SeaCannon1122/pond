#include "pond/pond.h"
#include <pond/pond.hpp>
#include <pond/data_types/motor_types.hpp>

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
    pond::Receiver<std::vector<MotorCommand>> command_receiver;
    pond::Receiver<std::vector<MotorFeedback>> feedback_receiver;

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

    feedback_receiver = createReceiver<std::vector<MotorFeedback>>({"get_motor_feedback"}, [this](std::vector<MotorFeedback>* feedbacks) {
        motors.resize(feedbacks->size());
        double time = pond::get_time();
        if (last_time == 0) last_time = time;

        for (uint32_t i = 0; i < feedbacks->size(); i++)
        {
            if (mode_pos)
            {
                feedbacks->at(i).pos = motors[i].pos;
                feedbacks->at(i).vel = 0;
            }
            if (mode_vel)
            {
                motors[i].pos += motors[i].vel * (time - last_time);

                feedbacks->at(i).pos = motors[i].pos;
                feedbacks->at(i).vel = motors[i].vel;
            }
            feedbacks->at(i).time = time;
            feedbacks->at(i).hw_time = time;
        }
        last_time = time;
    });

    command_receiver = createReceiver<std::vector<MotorCommand>>({"motor_cmd"}, [this](std::vector<MotorCommand>* commands) {
        motors.resize(commands->size());

        if (last_time == 0) last_time = pond::get_time();;
        for (uint32_t i = 0; i < commands->size(); i++)
        {
            if (commands->at(i).pos) motors[i].pos = *commands->at(i).pos;
            if (commands->at(i).vel) motors[i].vel = *commands->at(i).vel;
        }
    });

    return POND_SUCCESS;
}

void DummyMotor::onShutdown()
{
    feedback_receiver.destroy();
    command_receiver.destroy();
}
