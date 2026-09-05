#include "pond/pond.h"
#include <pond/pond.hpp>
#include <pond/data_types/motor_types.hpp>
#include <vector>

class DummyMotor : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
private:
    pond::Receiver<std::vector<MotorCommand>> receiver;
    pond::Distributor<std::vector<MotorFeedback>> distributor;
};

POND_MODULE_CPP_DECLARE(DummyMotor, "motor_tester", "module for testing motor")

pond_result DummyMotor::onStartup(const std::vector<void*>& args)
{
    distributor = createDistributor<std::vector<MotorFeedback>>({"motor_feedback"});
    receiver = createReceiver<std::vector<MotorCommand>>({"motor_cmd"}, [this](std::vector<MotorCommand>* commands) {
        
        std::vector<MotorFeedback> feedbacks(commands->size());
        
        for (uint32_t i = 0; i < commands->size(); i++)
        {
            feedbacks[i].pos = commands->at(i).pos;
            feedbacks[i].vel = commands->at(i).vel;

            distributor.distribute(feedbacks);
        }
    });
}

void DummyMotor::onShutdown()
{
    receiver.destroy();
    distributor.destroy();
}
