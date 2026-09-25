#include "pond_data_types/transform_types.hpp"
#include <Eigen/src/Core/Matrix.h>
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
        Pose2D received;
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
    Eigen::Vector2d min_body, max_body;
    std::string target_link;
    double timeout, max_angle_error;
};

POND_MODULE_CPP_DECLARE(ArmController, "arm_controller", "Control the robotic arm")

inline double safe_acos(double x) { return std::acos(std::clamp(x, -1.0, 1.0)); }

inline double law_cosines(double l1, double l2, double b) { return safe_acos((l1*l1 + l2*l2 - b*b)/(2*l1*l2));}


pond_result ArmController::onStartupTF(const std::vector<void*>& args)
{
    auto arm_joint_names_o = parameter("arm_joint_names").asStringArray().getStrict(3, 3);
    auto target_link_o = parameter("target_link_name").asString().getStrict();
    if (!arm_joint_names_o || !target_link_o) return POND_ERROR;
    target_link = *target_link_o;

    timeout = parameter("timeout").asDouble().get(1);
    max_angle_error = parameter("max_angle_error").asDouble().get(0.1);
    joint_states.resize(3);    

    GetJointInfoRequest joint_requests[3];
    for (uint32_t i = 0; i < 3; i++)
    {
        segments[i].joint_name = (*arm_joint_names_o)[i];
        joint_states[i].joint_name = segments[i].joint_name;

        if (!tfGetJointInfo(segments[i].joint_name, joint_requests[i])) return POND_ERROR;

        if (joint_requests[i].is_static) {POND_LOG("Joint '%s' is not dynamic", segments[i].joint_name.c_str()); return POND_ERROR;}
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

    min_body = segments[0].vec + Eigen::Rotation2Dd(segments[1].min) * segments[1].vec;
    max_body = segments[0].vec.normalized() * (segments[0].vec.norm() + segments[1].vec.norm());

    target.receiver = createReceiver<Pose2D>({"arm_target"}, [this](Pose2D* tf)
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
        double time = target.received.stamp.time;
        double target_angle = target.received.theta;
        Eigen::Vector2d end_pos(target.received.x, target.received.y);
        target.mutex.unlock();

        if (time == 0) return;

        double angles[3];
        
        Eigen::Vector2d body_target = end_pos - Eigen::Rotation2Dd(target_angle) * segments[2].vec;

        if (body_target.norm() < min_body.norm())
        {
            angles[0] = 
                atan2(end_pos.y(), end_pos.x()) + 
                law_cosines(end_pos.norm(), min_body.norm(), segments[2].vec.norm()) -
                atan2(min_body.y(), min_body.x()); 

            angles[1] = segments[1].min;

            angles[2] = 
                law_cosines(segments[1].vec.norm(), min_body.norm(), segments[0].vec.norm()) +
                law_cosines(min_body.norm(), segments[2].vec.norm(), end_pos.norm()) -
                atan2(segments[2].vec.y(), segments[2].vec.x()) -
                atan2(segments[1].vec.x(), segments[1].vec.y()) -
                M_PI/2.0;
            
        }
        else if (body_target.norm() > max_body.norm())
        {
            double angle_0_up = 
                atan2(end_pos.y(), end_pos.x()) + 
                law_cosines(end_pos.norm(), max_body.norm(), segments[2].vec.norm()) -
                atan2(max_body.y(), max_body.x());

            double angle_0_down = 
                atan2(end_pos.y(), end_pos.x()) -
                law_cosines(end_pos.norm(), max_body.norm(), segments[2].vec.norm()) -
                atan2(max_body.y(), max_body.x());
            
            angles[1] = atan2(segments[0].vec.y(), segments[0].vec.x()) - atan2(segments[1].vec.y(), segments[1].vec.x());

            double angle_2_up =
                law_cosines(max_body.norm(), segments[2].vec.norm(), end_pos.norm()) -
                atan2(segments[2].vec.y(), segments[2].vec.x()) -
                atan2(segments[1].vec.x(), segments[1].vec.y()) -
                M_PI/2.0;

            double angle_2_down =
                - law_cosines(max_body.norm(), segments[2].vec.norm(), end_pos.norm()) -
                atan2(segments[2].vec.y(), segments[2].vec.x()) -
                atan2(segments[1].vec.x(), segments[1].vec.y()) +
                3*M_PI/2.0;

            // if (
            //     std::abs(target_angle - angle_0_up - angles[1] - angle_2_up) > 
            //     std::abs(target_angle - angle_0_down - angles[1] - angle_2_down)
            // ) { angles[0] = angle_0_down; angles[2] = angle_2_down; }
            // else { 
                angles[0] = angle_0_up; angles[2] = angle_2_up; 
            //}
        }
        else
        {
            angles[0] = 
                atan2(body_target.y(), body_target.x()) + 
                law_cosines(segments[0].vec.norm(), body_target.norm(), segments[1].vec.norm()) -
                atan2(segments[0].vec.y(),  segments[0].vec.x());


            angles[1] = 
                law_cosines(segments[0].vec.norm(), segments[1].vec.norm(), body_target.norm()) -
                atan2(segments[1].vec.y(), segments[1].vec.x()) -
                atan2(segments[0].vec.x(), segments[0].vec.y()) -
                M_PI/2.0;

            angles[2] = - angles[0] - angles[1] + target_angle;
        }

        if (std::abs(angles[0] + angles[1] + angles[2] - target_angle) > max_angle_error) return;

        angles[0] = std::clamp(angles[0], segments[0].min, segments[0].max);
        angles[1] = std::clamp(angles[1], segments[1].min, segments[1].max);
        angles[2] = std::clamp(angles[2], segments[2].min, segments[2].max);
        
        
        ifs[0]->command.pos = angles[0];
        ifs[1]->command.pos = angles[1];
        ifs[2]->command.pos = angles[2];
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