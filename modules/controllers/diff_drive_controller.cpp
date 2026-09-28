#include "pond/pond.h"
#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>
#include <pond_data_types/command_types.hpp>
#include <pond_data_types/motor_types.hpp>

class DiffDriveController : public pond::ModuleBaseTF
{
public:
    virtual pond_result onStartupTF(const std::vector<void*>& args) override;
    virtual void onShutdownTF() override;
private:
    std::mutex mutex;
    TwistCommand cmd;

    pond::Receiver twist_receiver, update_receiver;
    std::vector<JointState> joint_states;

    std::vector<double> wheel_radii;
    double slip_multiplier;
    double timeout;
    bool timed_out = true;

    std::vector<double> wheel_y_offsets;
};

POND_MODULE_CPP_DECLARE(DiffDriveController, "diff_drive_controller", "Control a differential drive robot")

pond_result DiffDriveController::onStartupTF(const std::vector<void*>& args)
{
    std::string base_link = parameter("base_link").asString().get("base_link");
    slip_multiplier = parameter("slip_multiplier").asDouble().get(1.0);
    timeout = parameter("timeout").asDouble().get(0.5);

    auto space = parameterSpace("wheels");
    if (uint32_t motor_count = space.listLength("joint"); motor_count != 0)
    {
        joint_states.resize(motor_count);
        wheel_y_offsets.resize(motor_count);
        wheel_radii.resize(motor_count);
    }
    else POND_LOG_RETURN_ERROR("Did not find motor declaration");

    pond::ChannelsInfo channels_info;
    for (uint32_t i = 0; i < joint_states.size(); i++)
    {
        joint_states[i].joint_name = space.parameterAtIndex(i, "joint").asString().get("");
        
        auto radius = space.parameterAtIndex(i, "radius").asDouble().getStrict();
        auto motor_name = space.parameterAtIndex(i, "motor_name").asString().getStrict();
        if (!radius || !motor_name) return POND_ERROR;

        GetJointInfoRequest joint_request;
        if(!tfGetJointInfo(joint_states[i].joint_name, joint_request)) return POND_ERROR;

        GetFrameTransformRequest frame_request;
        if (!tfGetTransform(joint_request.parent_link_name, base_link, 0, frame_request)) return POND_ERROR;

        wheel_y_offsets[i] = (frame_request.tf * joint_request.tf).translation().y();
        wheel_radii[i] = *radius;
        channels_info.channel<MotorInterface>(*motor_name + "/update");
    }

    update_receiver = createReceiver<MotorInterface>(channels_info, [this](MotorInterface** ifs) {

        for (uint32_t i = 0; i < joint_states.size(); i++)
        {
            joint_states[i].angle = (ifs[i]->feedback.pos ? *ifs[i]->feedback.pos : 0);
            joint_states[i].time = ifs[i]->feedback.time;
            joint_states[i].hw_time = ifs[i]->feedback.hw_time;
        }

        tfSetJointStates(joint_states);

        mutex.lock();
        TwistCommand command = cmd;
        mutex.unlock();

        double time = pond::get_time();

        if (command.stamp.time + timeout < time)
        {
            if (!timed_out) POND_LOG("Timeout: last cmd stamp age (%f) > timeout (%f)", time-command.stamp.time, timeout);
            timed_out = true;
            for (int i = 0; i < joint_states.size(); i++) ifs[i]->command.vel = 0;
        }
        else
        {
            timed_out = false;
            for (int i = 0; i < joint_states.size(); i++)
                ifs[i]->command.vel = (cmd.lin[0] - cmd.ang[2] * slip_multiplier * wheel_y_offsets[i]) / wheel_radii[i];
        }
    });

    cmd.lin.setZero(); cmd.ang.setZero();

    twist_receiver = createReceiver<TwistCommand>({"cmd_vel"}, [this](TwistCommand* twist)
        {
            std::lock_guard<std::mutex> lock(mutex);
            cmd = *twist;
        }
    );

    return POND_SUCCESS;
}

void DiffDriveController::onShutdownTF()
{
    twist_receiver.destroy();
    update_receiver.destroy();
}