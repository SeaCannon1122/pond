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

    std::vector<std::string> joint_names;
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
    auto joint_names_o = parameter("wheel_joint_names").asStringArray().getStrict(1, 0);
    auto slip_multiplier_o = parameter("slip_multiplier").asDouble().getStrict();
    if (!joint_names_o || !slip_multiplier_o) return POND_ERROR;

    auto motor_names_o = parameter("motor_names").asStringArray().getStrict(joint_names_o->size(), joint_names_o->size());
    auto wheel_radii_o = parameter("wheel_radii").asDoubleArray().getStrict(joint_names_o->size(), joint_names_o->size());
    if (!wheel_radii_o || !motor_names_o) return POND_ERROR;

    timeout = parameter("timeout").asDouble().get(0.5);

    joint_names = std::move(*joint_names_o); wheel_radii = std::move(*wheel_radii_o); slip_multiplier = *slip_multiplier_o;
    wheel_y_offsets.reserve(joint_names.size());
    joint_states.reserve(joint_names.size());

    for (auto& name : joint_names)
    {
        joint_states.push_back({.joint_name = name});

        GetJointInfoRequest joint_request;
        if(!tfGetJointInfo(name, joint_request)) return POND_ERROR;

        if (joint_request.is_static)
        {
            POND_LOG("Joint '%s' is not dynamic", name.c_str());
            return POND_ERROR;
        }

        GetFrameTransformRequest frame_request;
        if (!tfGetTransform(joint_request.parent_link_name, base_link, 0, frame_request)) return POND_ERROR;

        Sophus::SE3d total = frame_request.tf * joint_request.tf;

        wheel_y_offsets.push_back(total.translation().y());
    }

    pond::ChannelsInfo channels_info;
    for (uint32_t i = 0; i < joint_names.size(); i++) channels_info.channel<MotorInterface>(motor_names_o->at(i) + "/update");

    update_receiver = createReceiver<MotorInterface>(channels_info, [this](MotorInterface** ifs) {

        for (uint32_t i = 0; i < joint_names.size(); i++)
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
            for (int i = 0; i < joint_names.size(); i++) ifs[i]->command.vel = 0;
        }
        else
        {
            timed_out = false;
            for (int i = 0; i < joint_names.size(); i++)
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