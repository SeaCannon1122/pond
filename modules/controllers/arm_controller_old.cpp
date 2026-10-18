#include "pond/hpp/module_base.hpp"
#include "pond_data_types/transform_types.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>
#include <pond_data_types/command_types.hpp>
#include <pond_data_types/motor_types.hpp>

struct segment
{
    std::string joint_name;
    double min;
    double max;

    Eigen::Vector2d vec;
    MotorCommand motor_command;
};

class ArmControllerOld : public pond::ModuleBaseTF
{
public:
    virtual pond_result onStartupTF(const std::vector<void*>& args) override;
    virtual void onShutdownTF() override;
private:

    struct
    {
        pond::Callback callback;
        Pose2D received;
        std::mutex mutex;
    } target;

    struct
    {
        pond::Callback callback;
        std::atomic<double> received{0.02};
    } arm_speed;

    pond::Callback update_callback;

    std::vector<JointState> joint_states;

    segment segments[3];

    std::string target_link;
    double timeout;
};

POND_MODULE_CPP_DECLARE(ArmControllerOld, "arm_controller_old", "Control the robotic arm like in quac")

inline double safe_acos(double x) { return std::acos(std::clamp(x, -1.0, 1.0)); }
inline double law_cosines(double l1, double l2, double b) { return safe_acos((l1*l1 + l2*l2 - b*b)/(2*l1*l2));}


pond_result ArmControllerOld::onStartupTF(const std::vector<void*>& args)
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
        if (!joint_requests[i].max_angle || !joint_requests[i].min_angle) {POND_LOG("Joint '%s' does not have limits set", segments[i].joint_name.c_str()); return POND_ERROR;}

        segments[i].min = *joint_requests[i].min_angle; segments[i].max = *joint_requests[i].max_angle;
    }
    
    for (uint32_t i = 0; i < 3; i++)
    {
        GetFrameTransformRequest tf_request;
        if (!tfGetTransform(i == 2 ? target_link : joint_requests[i+1].parent_link_name, joint_requests[i].child_link_name, 0, tf_request)) return POND_ERROR;
        Sophus::SE3d total = (i == 2 ? tf_request.tf : tf_request.tf * joint_requests[i+1].tf);
        segments[i].vec = Eigen::Vector2d(total.translation().x(), total.translation().y());
    }

    target.callback = createCallback<Pose2D>({"arm_target"}, [this](Pose2D* tf)
        {
            if (tf->stamp.time + timeout < pond::get_time()) return;
            if (!std::isfinite(tf->theta) || !std::isfinite(tf->x) || !std::isfinite(tf->y))
            {
                POND_LOG("Got invalid angles");
                return;
            }

            std::lock_guard<std::mutex> lock(target.mutex);
            target.received = *tf;
        }
    );

    arm_speed.callback = createCallback<double>({"arm_speed"}, [this](double* speed) {arm_speed.received.store(*speed);});

    pond::ChannelsInfo info;
    info.channels<MotorCommand*>("control_arm/arm_motor0_command", "control_arm/arm_motor1_command", "control_arm/arm_motor2_command");
    info.channels<MotorFeedback*>("control_arm/arm_motor0_feedback", "control_arm/arm_motor1_feedback", "control_arm/arm_motor2_feedback");
    
    update_callback = createCallback<void>(info, [this](void** data) {
    
        MotorCommand*** commands = (MotorCommand***)data;
        MotorFeedback*** feedbacks = (MotorFeedback***)&data[3];
        
        for (uint32_t i = 0; i < 3; i++) if (*feedbacks[i] == NULL) POND_LOG_RETURN("%s is NULL", ("control_arm/arm_motor"+ std::to_string(i) +"_feedback").c_str());
        for (uint32_t i = 0; i < 3; i++) *commands[i] = &segments[i].motor_command;
        
        for (uint32_t i = 0; i < 3; i++)
        {
            joint_states[i].angle = (feedbacks[i][0]->pos ? *feedbacks[i][0]->pos : 0);
            joint_states[i].time = feedbacks[i][0]->time;
            joint_states[i].hw_time = feedbacks[i][0]->hw_time;
        }

        tfSetJointStates(joint_states);

        target.mutex.lock();
        double time = target.received.stamp.time;
        double target_angle = target.received.theta;
        Eigen::Vector2d end_pos(target.received.x, target.received.y);
        target.mutex.unlock();

        if (time == 0) return;

        double angles[3];
    
        angles[0] = 
            atan2(end_pos.y(), end_pos.x()) + 
            law_cosines(segments[0].vec.norm(), end_pos.norm(), segments[1].vec.norm()) -
            atan2(segments[0].vec.y(),  segments[0].vec.x());

        angles[1] = 
            law_cosines(segments[0].vec.norm(), segments[1].vec.norm(), end_pos.norm()) -
            atan2(segments[1].vec.y(), segments[1].vec.x()) -
            atan2(segments[0].vec.x(), segments[0].vec.y()) -
            M_PI/2.0;

        angles[2] = - angles[0] - angles[1] + target_angle;

        for (uint32_t i = 0; i < 3; i++) segments[i].motor_command.pos = std::clamp(angles[i], segments[i].min, segments[i].max);
    });

    return POND_SUCCESS;
}

void ArmControllerOld::onShutdownTF()
{
    update_callback.destroy();
    arm_speed.callback.destroy();
    target.callback.destroy();
}