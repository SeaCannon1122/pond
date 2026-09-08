#include "pond/pond.h"
#include <pond/pond.hpp>
#include <pond/data_types/motor_types.hpp>

struct motor
{
    double pos = 0;
    double last_vel = 0;
};

class DummyMotor : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
private:
    pond::Receiver<std::vector<MotorCommand>> receiver;
    pond::Distributor<std::vector<MotorFeedback>> distributor;

    bool mode_pos;
    bool mode_vel;
    std::vector<motor> motors;
    double last_time = 0;
    std::vector<MotorFeedback> feedbacks;
};

POND_MODULE_CPP_DECLARE(DummyMotor, "dummy_motor", "mock motor")

pond_result DummyMotor::onStartup(const std::vector<void*>& args)
{
    auto mode_o = parameter("mode").asString().getStrict({"position", "velocity"});
    if (!mode_o) return POND_ERROR;

    mode_pos = (*mode_o == "position");
    mode_vel = (*mode_o == "velocity");

    distributor = createDistributor<std::vector<MotorFeedback>>({"motor_feedback"});
    receiver = createReceiver<std::vector<MotorCommand>>({"motor_cmd"}, [this](std::vector<MotorCommand>* commands) {
        
        double time = pond::get_time();

        feedbacks.resize(commands->size());
        motors.resize(commands->size());
        
        for (uint32_t i = 0; i < commands->size(); i++)
        {
            if (mode_pos) feedbacks[i].pos = commands->at(i).pos;
            if (mode_vel)
            {
                feedbacks[i].pos += motors[i].last_vel * (time - last_time);
                feedbacks[i].vel = commands->at(i).vel;
                motors[i].last_vel = feedbacks[i].vel;
            }
            feedbacks[i].time = time;
            feedbacks[i].hw_time = time;
        }

        distributor.distribute(feedbacks);
        last_time = time;
    });

    return POND_SUCCESS;
}

void DummyMotor::onShutdown()
{
    receiver.destroy();
    distributor.destroy();
}
