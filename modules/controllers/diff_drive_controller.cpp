#include "pond/pond.h"
#include <pond/pond.hpp>
#include <pond/data_types/command_types.hpp>
#include <pond/data_types/motor_types.hpp>
#include <pond/data_types/robot_state_types.hpp>
#include <mutex>
#include <vector>

class DiffDriveController : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
    std::mutex mutex;
    TwistCommand cmd;

    pond::Receiver<TwistCommand> twist_receiver;

    pond::Distributor<std::vector<MotorCommand>> command_distributor;
    pond::Receiver<std::vector<MotorFeedback>> feedback_receiver;

    std::vector<MotorCommand> motor_commands;

    std::vector<std::string> joint_names;
    std::vector<double> wheel_radii;
    double slip_multiplier;

    std::vector<double> wheel_y_offsets;
};

POND_MODULE_CPP_DECLARE(DiffDriveController, "diff_drive_controller", "Control a differential drive robot")

pond_result DiffDriveController::onStartup(const std::vector<void*>& args)
{
    std::string base_link = parameter("base_link").asString().get("base_link");
    auto joint_names_o = parameter("wheel_joint_names").asStringArray().getStrict(1, 0);
    auto slip_multiplier_o = parameter("slip_multiplier").asDouble().getStrict();
    if (!joint_names_o || !slip_multiplier_o) return POND_ERROR;

    auto wheel_radii_o = parameter("wheel_radii").asDoubleArray().getStrict((*joint_names_o).size(), (*joint_names_o).size());
    if (!wheel_radii_o) return POND_ERROR;

    joint_names = std::move(*joint_names_o); wheel_radii = std::move(*wheel_radii_o); slip_multiplier = *slip_multiplier_o;
    wheel_y_offsets.reserve(joint_names.size());
    motor_commands.resize(joint_names.size());

    pond::Distributor<GetJointInfoRequest> joint_requester = createDistributor<GetJointInfoRequest>({"get_robot_joint_info"});
    pond::Distributor<GetFrameTransformRequest> tf_requester = createDistributor<GetFrameTransformRequest>({"get_robot_transform"});

    for (auto& name : joint_names)
    {
        GetJointInfoRequest joint_request;
        joint_request.joint_name = name;
        joint_requester.distribute(joint_request);

        if (!joint_request.fufilled || joint_request.is_static)
        {
            if (!joint_request.fufilled) POND_LOG("Could not retrieve information about joint '%s'", name.c_str());
            else if (joint_request.is_static) POND_LOG("Joint '%s' is not dynamic", name.c_str());
            tf_requester.destroy();
            joint_requester.destroy();
            return POND_ERROR;
        }

        GetFrameTransformRequest frame_request;
        frame_request.source_frame = base_link;
        frame_request.target_frame = joint_request.child_link_name;
        frame_request.time_point = 0;
        tf_requester.distribute(frame_request);

        if (!joint_request.fufilled)
        {
            POND_LOG("Could not retrieve transform from '%s' to '%s'", frame_request.source_frame.c_str(), frame_request.target_frame.c_str());
            tf_requester.destroy();
            joint_requester.destroy();
            return POND_ERROR;
        }

        Sophus::SE3d total = frame_request.tf * joint_request.tf;

        wheel_y_offsets.push_back(total.translation().y());
    }

    tf_requester.destroy();
    joint_requester.destroy();

    command_distributor = createDistributor<std::vector<MotorCommand>>({"motor_cmd"});
    feedback_receiver = createReceiver<std::vector<MotorFeedback>>({"motor_feedback"}, [this](std::vector<MotorFeedback>* feedback)
        {

        }
    );

    twist_receiver = createReceiver<TwistCommand>({"cmd_vel"}, [this](TwistCommand* twist)
        {
            std::lock_guard<std::mutex> lock(mutex);
            cmd = *twist;
        }
    );

    return POND_SUCCESS;
}

void DiffDriveController::onShutdown()
{
    feedback_receiver.destroy();
    twist_receiver.destroy();
    command_distributor.destroy();
}

void DiffDriveController::onFrame()
{
    mutex.lock();
    TwistCommand command = cmd;
    mutex.unlock();

    for (int i = 0; i < joint_names.size(); i++)
        motor_commands[i].vel = (cmd.lin[0] - cmd.ang[2] * slip_multiplier * wheel_y_offsets[i]) / wheel_radii[i];

    command_distributor.distribute(motor_commands);
}
