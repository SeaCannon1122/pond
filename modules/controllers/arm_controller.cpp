#include <pond/module_base_tf.hpp>
#include <pond/data_types/command_types.hpp>
#include <pond/data_types/motor_types.hpp>
#include <pond/data_types/robot_state_types.hpp>

struct segment
{
    std::string joint_name;
    double min;
    double max;

    double x;
    double y;
};

class ArmController : public pond::ModuleBaseTF
{
public:
    virtual pond_result onStartupTF(const std::vector<void*>& args) override;
    virtual void onShutdownTF() override;
    virtual void onFrame() override;
private:

    struct
    {
        pond::Receiver<FrameTransform> receiver;
        FrameTransform received;
        std::mutex mutex;
    } target;

    struct
    {
        pond::Receiver<double> receiver;
        std::atomic<double> received{0.02};
    } arm_speed;

    struct
    {
        pond::Receiver<bool> receiver;
        std::atomic<bool> received{false};
        bool disabled = false;
    } disable_arm;

    struct
    {
        pond::Receiver<double> width_receiver;
        std::atomic<double> received_width{-1};
        double radius, offset;
    } gripper;

    pond::Distributor<std::vector<MotorCommand>> command_distributor;
    pond::Distributor<std::vector<MotorFeedback>> feedback_distributor;

    std::vector<MotorCommand> motor_commands;
    std::vector<MotorFeedback> motor_feedback;
    std::vector<JointState> joint_states;

    segment segments[3];
    std::string target_link;
    double timeout;
};

POND_MODULE_CPP_DECLARE(ArmController, "arm_controller", "Control the robotic arm")

pond_result ArmController::onStartupTF(const std::vector<void*>& args)
{
    auto arm_joint_names_o = parameter("arm_joint_names").asStringArray().getStrict(3, 3);
    auto gripper_joint_o = parameter("gripper_joint_name").asString().getStrict();
    auto target_link_o = parameter("target_link_name").asString().getStrict();
    if (!arm_joint_names_o || !target_link_o || !gripper_joint_o) return POND_ERROR;
    target_link = *target_link_o;

    gripper.radius = parameter("gripper_radius").asDouble().get(0.1);
    gripper.offset = parameter("gripper_offset").asDouble().get(0.1);
    timeout = parameter("timeout").asDouble().get(1);

    joint_states.resize(4);
    motor_commands.resize(4);
    motor_feedback.resize(4);
    
    joint_states[3].joint_name = *gripper_joint_o;

    GetJointInfoRequest joint_requests[3];
    for (uint32_t i = 0; i < 3; i++)
    {
        segments[i].joint_name = (*arm_joint_names_o)[i];
        joint_states[i].joint_name = segments[i].joint_name;

        if (!tfGetJointInfo(segments[i].joint_name, joint_requests[i])) return POND_ERROR;

        if (joint_requests[i].is_static) {POND_LOG("Joint '%s' is not dynamic", segments[i].joint_name.c_str()); return POND_ERROR;}
        if (!joint_requests[i].max_angle.has_value() || !joint_requests[i].min_angle.has_value()) {POND_LOG("Joint '%s' does not have limits set", segments[i].joint_name.c_str()); return POND_ERROR;}
    }
    for (uint32_t i = 0; i < 3; i++)
    {
        GetFrameTransformRequest tf_request;
        if (!tfGetTransform(i == 2 ? target_link : joint_requests[i+1].parent_link_name, joint_requests[i].child_link_name, 0, tf_request)) return POND_ERROR;
        Sophus::SE3d total = (i == 2 ? tf_request.tf : tf_request.tf * joint_requests[i].tf);
        segments[i].x = total.translation().x();
        segments[i].y = total.translation().y();
    }

    command_distributor = createDistributor<std::vector<MotorCommand>>({"motor_cmd"});
    feedback_distributor = createDistributor<std::vector<MotorFeedback>>({"get_motor_feedback"});

    target.receiver = createReceiver<FrameTransform>({"arm_target"}, [this](FrameTransform* tf)
        {
            if (tf->stamp.time + timeout < pond::get_time()) return;

            std::lock_guard<std::mutex> lock(target.mutex);
            target.received = *tf;
        }
    );

    gripper.width_receiver = createReceiver<double>({"gripper_width"}, [this](double* width) {gripper.received_width.store(*width);});
    arm_speed.receiver = createReceiver<double>({"arm_speed"}, [this](double* speed) {arm_speed.received.store(*speed);});
    disable_arm.receiver = createReceiver<bool>({"disable_arm"}, [this](bool* disable) {disable_arm.received.store(*disable);});

    return POND_SUCCESS;
}

void ArmController::onShutdownTF()
{
    disable_arm.receiver.destroy();
    arm_speed.receiver.destroy();
    target.receiver.destroy();
    gripper.width_receiver.destroy();
    feedback_distributor.destroy();
    command_distributor.destroy();
}

void ArmController::onFrame()
{
    feedback_distributor.distribute(motor_feedback);

    for (uint32_t i = 0; i < motor_feedback.size(); i++)
    {
        joint_states[i].angle = (motor_feedback[i].pos ? *motor_feedback[i].pos : 0);
        joint_states[i].time = motor_feedback[i].time;
        joint_states[i].hw_time = motor_feedback[i].hw_time;
    }

    tfSetJointStates(joint_states);

    if (disable_arm.received.load())
    {
        if (disable_arm.disabled == false)
        {
            disable_arm.disabled = true;
            for (uint32_t i = 0; i < 4; i++) motor_commands[i].disable = true;
            command_distributor.distribute(motor_commands);
        }
        return;
    }

    if (disable_arm.disabled)
    {
        disable_arm.disabled = false;
        for (uint32_t i = 0; i < 4; i++) motor_commands[i].disable = false;
    }
    else for (uint32_t i = 0; i < 4; i++) motor_commands[i].disable = std::nullopt;

    target.mutex.lock();
    FrameTransform target_tf = target.received;
    target.mutex.unlock();
    
    //command_distributor.distribute(motor_commands);
}
