#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>
#include <pond_data_types/motor_types.hpp>

class LegKinematicsController : public pond::ModuleBaseTF
{
public:
    virtual pond_result onStartupTF(const std::vector<void*>& args) override;
    virtual void onShutdownTF() override;
private:
    double limb_length_hip_to_shoulder;
    double limb_length_thigh;
    double limb_length_shin;
    
    double half_length, half_width;

    std::vector<JointState> joint_states;
    std::vector<MotorCommand> commands;

    pond::Callback stage_callback;
};

POND_MODULE_CPP_DECLARE(LegKinematicsController, "leg_kinematics_controller", "calculating the target leg angles")

pond_result LegKinematicsController::onStartupTF(const std::vector<void*>& args)
{
    auto joint_names = parameter("joint_names").asStringArray().getStrict(12, 12);
    auto motor_names = parameter("motor_names").asStringArray().getStrict(12, 12);
    auto foot_link_names = parameter("foot_link_names").asStringArray().getStrict(4, 4);
    auto reference_link = parameter("reference_link").asString().getStrict();
    if (!joint_names || !motor_names || !foot_link_names || !reference_link) return POND_ERROR;

    commands.resize(12);
    joint_states.resize(12);
    for (uint32_t i = 0; i < 12; i++) joint_states[i].joint_name = joint_names->at(i);

    GetJointInfoRequest joint_requests[3];
    GetFrameTransformRequest tf_requests[4];

    for (uint32_t i = 0; i < 3; i++) if (!tfGetJointInfo(joint_names->at(i), joint_requests[i])) return POND_ERROR;
    
    if (!tfGetTransform(joint_requests[1].parent_link_name, joint_requests[0].child_link_name, 0, tf_requests[0])) return POND_ERROR;
    if (!tfGetTransform(joint_requests[2].parent_link_name, joint_requests[1].child_link_name, 0, tf_requests[1])) return POND_ERROR;
    if (!tfGetTransform(foot_link_names->at(0), joint_requests[2].child_link_name, 0, tf_requests[2])) return POND_ERROR;

    limb_length_hip_to_shoulder =   (tf_requests[0].tf * joint_requests[1].tf).translation().norm();
    limb_length_thigh =             (tf_requests[1].tf * joint_requests[2].tf).translation().norm();
    limb_length_shin =              (tf_requests[2].tf).translation().norm();

    if (!tfGetTransform(joint_requests[0].parent_link_name, *reference_link, 0, tf_requests[3])) return POND_ERROR;
    Sophus::SE3d total = tf_requests[3].tf * joint_requests[0].tf;

    half_length = total.translation().x()/2.0;
    half_width = total.translation().y()/2.0;

    pond::ChannelsInfo info;
    info.channel<std::vector<Eigen::Vector3d>*>("calculate_joint_angles/foot_target_positions");
    
    for (uint32_t i = 0; i < 12; i++) info.channel<MotorCommand*>("calculate_joint_angles/" + motor_names->at(i) + "_command");

    stage_callback = createCallback<void>(info, [this](void** data)
        {
            std::vector<Eigen::Vector3d>** foot_target_positions_in = (std::vector<Eigen::Vector3d>**)data[0];
            MotorCommand*** commands_out = (MotorCommand***)&data[1];

            if (*foot_target_positions_in == NULL) POND_LOG_RETURN("calculate_joint_angles/foot_target_positions is NULL");
            for (uint32_t i = 0; i < 12; i++) *commands_out[i] = &commands[i];

            auto law_of_cosines = +[](double l1, double l2, double b, double* theta) -> bool
            {
                double acos_theta = (l1*l1 + l2*l2 - b*b)/(2*l1*l2);
                if (acos_theta > 1.0 || acos_theta < -1.0) return false;

                *theta = acos(acos_theta);

                return true;
            };

            for(size_t i = 0; i < 4; i++)
            {
                double x = foot_target_positions_in[0][0][i][0] - (i < 2 ? half_length: -half_length);
                double y = foot_target_positions_in[0][0][i][1] - (i % 2 == 0 ? half_width : -half_width);
                double z = foot_target_positions_in[0][0][i][2];

                if (i % 2 != 0) y = -y;
                if (z > 0) x*= -1;

                double knee_angle, shoulder_angle, hip_angle;

                double shoulder_to_foot_sq = z * z + y * y - limb_length_hip_to_shoulder * limb_length_hip_to_shoulder;
                if (shoulder_to_foot_sq < 0) continue;
                double shoulder_to_foot = std::sqrt(shoulder_to_foot_sq);

                hip_angle = std::atan2(y, std::abs(z)) + std::atan2(shoulder_to_foot, limb_length_hip_to_shoulder);

                double distance = std::sqrt(x * x + shoulder_to_foot * shoulder_to_foot);

                if (!law_of_cosines(limb_length_thigh, limb_length_shin, distance, &knee_angle)) continue;
                if (!law_of_cosines(limb_length_thigh, distance, limb_length_shin, &shoulder_angle)) continue;

                shoulder_angle -= std::atan2(x, shoulder_to_foot);
                if(z>0) shoulder_angle += M_PI;

                // Adjust angle from triangle perspective to robot perspective
                hip_angle -= M_PI_2;
                if (z>0) hip_angle *= -1;
                knee_angle -= M_PI;

                if (i%2) hip_angle *= -1;

                commands[3*i+0].pos = hip_angle;
                commands[3*i+1].pos = shoulder_angle;
                commands[3*i+2].pos = knee_angle;
            }
        }
    );

    return POND_SUCCESS;
}

void LegKinematicsController::onShutdownTF()
{
    stage_callback.destroy();
}
