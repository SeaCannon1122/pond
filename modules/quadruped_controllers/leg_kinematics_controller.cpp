#include <algorithm>
#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>
#include <pond_data_types/trajectory_types.hpp>

#include "quadruped_robot_config.hpp"

class LegKinematicsController : public pond::ModuleBaseTF
{
public:
    virtual pond_result onStartupTF(const std::vector<void*>& args) override;
    virtual void onShutdownTF() override;
private:
    QuadrupedRobotConfig cfg;

    std::array<Eigen::Vector3d, 4> target_positions;

    pond::Callback stage_callback;
};

POND_MODULE_CPP_DECLARE(LegKinematicsController, "leg_kinematics_controller", "calculating the target leg angles")

pond_result LegKinematicsController::onStartupTF(const std::vector<void*>& args)
{
    if (!cfg.init(*this)) return POND_ERROR;

    stage_callback = createCallback<std::vector<TrajectoryPoint>*, std::array<Eigen::Vector3d, 4>*>(
        {"calculate_joint_angles/target_trajectory_states", "calculate_joint_angles/joint_target_positions"},
        [this](std::vector<TrajectoryPoint>** trajectory_states_in, std::array<Eigen::Vector3d, 4>** joint_target_positions_out)
        {
            if (*trajectory_states_in == NULL) POND_LOG_RETURN("calculate_joint_angles/target_trajectory_states is NULL");
            *joint_target_positions_out = &target_positions;

            auto law_of_cosines = +[](double l1, double l2, double b, double* theta) -> bool
            {
                double acos_theta = (l1*l1 + l2*l2 - b*b)/(2*l1*l2);
                if (acos_theta > 1.0 || acos_theta < -1.0) return false;

                *theta = acos(acos_theta);

                return true;
            };

            for(size_t i = 0; i < 4; i++)
            {
                Eigen::Vector3d target = trajectory_states_in[0][0][i].pose.translation() - cfg.legs[i].origin_pos;

                double limb_length_hip_to_shoulder = std::abs(cfg.legs[i].segments[0].offset.y() + cfg.legs[i].segments[1].offset.y() + cfg.legs[i].segments[2].offset.y());
                double limb_length_thigh = std::abs(cfg.legs[i].segments[1].offset.z());
                double limb_length_shin = std::abs(cfg.legs[i].segments[2].offset.z());

                if (i%2) target.y() *= -1;

                double knee_angle, shoulder_angle, hip_angle;

                double shoulder_to_foot_sq = target.z() * target.z() + target.y() * target.y() - limb_length_hip_to_shoulder * limb_length_hip_to_shoulder;
                if (shoulder_to_foot_sq < 0) continue;
                double shoulder_to_foot = std::sqrt(shoulder_to_foot_sq);

                hip_angle = std::atan2(target.y(), -target.z()) + std::atan2(shoulder_to_foot, limb_length_hip_to_shoulder);

                double distance = std::sqrt(target.x() * target.x() + shoulder_to_foot * shoulder_to_foot);

                if (!law_of_cosines(limb_length_thigh, limb_length_shin, distance, &knee_angle)) continue;
                if (!law_of_cosines(limb_length_thigh, distance, limb_length_shin, &shoulder_angle)) continue;

                shoulder_angle -= std::atan2(target.x(), shoulder_to_foot);

                // Adjust angle from triangle perspective to robot perspective
                hip_angle -= M_PI_2;
                knee_angle -= M_PI;

                if (i%2) hip_angle *= -1;

                target_positions[i] = Eigen::Vector3d(hip_angle, shoulder_angle, knee_angle);
                for (uint32_t j = 0; j < 3; j++) target_positions[i][j] = std::clamp(target_positions[i][j], cfg.legs[i].segments[j].joint_min, cfg.legs[i].segments[j].joint_max);
            }
        }
    );

    return POND_SUCCESS;
}

void LegKinematicsController::onShutdownTF()
{
    stage_callback.destroy();
}
