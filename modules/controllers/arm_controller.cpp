#include "pond_data_types/transform_types.hpp"
#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>
#include <pond_data_types/command_types.hpp>
#include <pond_data_types/motor_types.hpp>

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
private:

    struct
    {
        pond::Receiver receiver;
        FrameTransform received;
        std::mutex mutex;
    } target;

    struct
    {
        pond::Receiver receiver;
        std::atomic<double> received{0.02};
    } arm_speed;

    struct
    {
        pond::Receiver receiver;
        std::atomic<bool> received{false};
        bool disabled = false;
    } disable_arm;

    pond::Receiver update_receiver;

    std::vector<JointState> joint_states;

    segment segments[3];
    std::string target_link;
    double timeout;
};

POND_MODULE_CPP_DECLARE(ArmController, "arm_controller", "Control the robotic arm")

pond_result ArmController::onStartupTF(const std::vector<void*>& args)
{
    auto arm_joint_names_o = parameter("arm_joint_names").asStringArray().getStrict(3, 3);
    auto target_link_o = parameter("target_link_name").asString().getStrict();
    if (!arm_joint_names_o || !target_link_o) return POND_ERROR;
    target_link = *target_link_o;

    timeout = parameter("timeout").asDouble().get(1);
    joint_states.resize(3);    

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

    target.receiver = createReceiver<FrameTransform>({"arm_target"}, [this](FrameTransform* tf)
        {
            if (tf->stamp.time + timeout < pond::get_time()) return;

            std::lock_guard<std::mutex> lock(target.mutex);
            target.received = *tf;
        }
    );

    arm_speed.receiver = createReceiver<double>({"arm_speed"}, [this](double* speed) {arm_speed.received.store(*speed);});
    disable_arm.receiver = createReceiver<bool>({"disable_arm"}, [this](bool* disable) {disable_arm.received.store(*disable);});

    update_receiver = createReceiver<MotorInterface>(pond::ChannelsInfo().channels<MotorInterface, MotorInterface, MotorInterface>("arm_motor0/update", "arm_motor1/update", "arm_motor2/update"), [this](MotorInterface** ifs) {
    
        for (uint32_t i = 0; i < 3; i++)
        {
            joint_states[i].angle = (ifs[i]->feedback.pos ? *ifs[i]->feedback.pos : 0);
            joint_states[i].time = ifs[i]->feedback.time;
            joint_states[i].hw_time = ifs[i]->feedback.hw_time;
        }

        tfSetJointStates(joint_states);

        target.mutex.lock();
        FrameTransform target_tf = target.received;
        target.mutex.unlock();

        double target_x = msg->position.x; double target_y = msg->position.z;

        double seg_0 = sqrt(segments[0].x * segments[0].x + segments[0].y * segments[0].y);
        double seg_1 = sqrt(segments[1].x * segments[1].x + segments[1].y * segments[1].y);
        double target = sqrt(target_x * target_x + target_y * target_y);

        if (seg_0 + seg_1 < target)
        {
        double factor = (seg_0 + seg_1) / target;
        target_x *= factor;
        target_y *= factor;
        target *= factor;
        }

        double angle_0 = atan2(target_y, target_x) + safeAcos((seg_0 * seg_0 + target * target - seg_1 * seg_1) / (2.0 * seg_0 * target));
        double angle_1 = safeAcos((seg_1 * seg_1 + seg_0 * seg_0 - target * target) / (2.0 * seg_0 * seg_1)) - M_PI;

        double joint_angle_0 = angle_0 - atan2(segments[0].y,  segments[0].x);
        double joint_angle_1 = angle_1 - atan2(segments[1].y,  segments[1].x) + atan2(segments[0].y,  segments[0].x);

        joint_trajectory.joint_names.push_back(segments[0].joint);
        joint_trajectory.joint_names.push_back(segments[1].joint);

        point.positions.push_back(joint_angle_0);
        point.positions.push_back(joint_angle_1);

        if (segments.size() == 3)
        {
        joint_trajectory.joint_names.push_back(segments[2].joint);
        point.positions.push_back(- joint_angle_0 - joint_angle_1 + msg->orientation.y);
        }

        joint_trajectory.points.push_back(point);
        joint_trajectory_publisher->publish(joint_trajectory);
    });

    return POND_SUCCESS;
}

void ArmController::onShutdownTF()
{
    update_receiver.destroy();
    disable_arm.receiver.destroy();
    arm_speed.receiver.destroy();
    target.receiver.destroy();
}