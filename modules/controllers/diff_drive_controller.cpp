#include "pond/hpp/module_base.hpp"
#include "pond/pond.h"
#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>
#include <pond_data_types/command_types.hpp>
#include <pond_data_types/motor_types.hpp>
#include <vector>

struct wheel
{
    double radius;
    double y_offsets;
    MotorCommand motor_command;
    std::string motor_name;
};

class DiffDriveController : public pond::ModuleBaseTF
{
public:
    virtual pond_result onStartupTF(const std::vector<void*>& args) override;
    virtual void onShutdownTF() override;
private:
    std::mutex mutex;
    TwistCommand cmd;

    pond::Callback twist_callback, update_callback;
    std::vector<JointState> joint_states;

    double slip_multiplier;
    double timeout;
    bool timed_out = true;

    std::vector<wheel> wheels;
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
        wheels.resize(motor_count);
    }
    else POND_LOG_RETURN_ERROR("Did not find motor declaration");

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

        wheels[i].y_offsets = (frame_request.tf * joint_request.tf).translation().y();
        wheels[i].radius = *radius;
        wheels[i].motor_name = *motor_name;
    }

    pond::ChannelsInfo info;
    for (auto& wheel : wheels) info.channel<void>("control_wheels/" + wheel.motor_name + "_command");
    for (auto& wheel : wheels) info.channel<void>("control_wheels/" + wheel.motor_name + "_feedback");

    update_callback = createCallback<void>(info, [this](void** data) {

        MotorCommand*** commands = (MotorCommand***)data;
        MotorFeedback*** feedbacks = (MotorFeedback***)&data[wheels.size()];

        for (uint32_t i = 0; i < wheels.size(); i++) if (*feedbacks[i] == NULL) POND_LOG_RETURN("%s is NULL", ("control_wheels/" + wheels[i].motor_name + "_feedback").c_str());
        for (uint32_t i = 0; i < wheels.size(); i++) *commands[i] = &wheels[i].motor_command;

        for (uint32_t i = 0; i < wheels.size(); i++)
        {
            joint_states[i].angle = (feedbacks[i][0]->pos ? *feedbacks[i][0]->pos : 0);
            joint_states[i].time = feedbacks[i][0]->time;
            joint_states[i].hw_time = feedbacks[i][0]->hw_time;
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
            for (int i = 0; i < wheels.size(); i++) commands[i][0]->vel = 0;
        }
        else
        {
            timed_out = false;
            for (int i = 0; i < wheels.size(); i++) commands[i][0]->vel = (cmd.lin[0] - cmd.ang[2] * slip_multiplier * wheels[i].y_offsets) / wheels[i].radius;
        }
    });

    cmd.lin.setZero(); cmd.ang.setZero();

    twist_callback = createCallback<TwistCommand>({"cmd_vel"}, [this](TwistCommand* twist)
        {
            std::lock_guard<std::mutex> lock(mutex);
            cmd = *twist;
        }
    );

    return POND_SUCCESS;
}

void DiffDriveController::onShutdownTF()
{
    twist_callback.destroy();
    update_callback.destroy();
}