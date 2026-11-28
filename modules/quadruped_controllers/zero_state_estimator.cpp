// logic copied from https://github.com/mit-biomimetics/Cheetah-Software/blob/master/common/src/Controllers/PositionVelocityEstimator.cpp

#include <Eigen/src/Core/Matrix.h>
#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>

#include <pond_data_types/trajectory_types.hpp>
#include <pond_data_types/imu_types.hpp>

#include "quadruped_robot_config.hpp"

using Matrix12x12 = Eigen::Matrix<double, 12, 12>;
using Matrix18x3  = Eigen::Matrix<double, 18, 3>;
using Matrix18x18 = Eigen::Matrix<double, 18, 18>;
using Matrix28x18 = Eigen::Matrix<double, 28, 18>;
using Matrix28x28 = Eigen::Matrix<double, 28, 28>;
using Vector12    = Eigen::Matrix<double, 12, 1>;
using Vector18    = Eigen::Matrix<double, 18, 1>;
using Vector28    = Eigen::Matrix<double, 28, 1>;

class ZeroStateEstimator : public pond::ModuleBaseTF
{
public:
    virtual pond_result onStartupTF(const std::vector<void*>& args) override;
    virtual void onShutdownTF() override;
private:

    struct
    {
        struct { double foot_position, foot_velocity, foot_height; } sensor;
        struct { double foot_position, imu_position, imu_velocity; } process;
    } noise;

    double frame_dt;
    QuadrupedRobotConfig cfg;

    pond::Callback stage_callback;
    TrajectoryPoint trajectory_point;
};

POND_MODULE_CPP_DECLARE(ZeroStateEstimator, "zero_state_estimator", "estimates robot state")

pond_result ZeroStateEstimator::onStartupTF(const std::vector<void*>& args)
{
    if (!cfg.init(*this)) return POND_ERROR;

    frame_dt = parameter("frame_dt").asDouble().get(0.001);

    stage_callback = createCallback<std::array<Eigen::Vector3d, 4>*, std::array<Eigen::Vector3d, 4>*, ImuData*, TrajectoryPoint*>(
        {"estimate_state/joint_positions", "estimate_state/joint_velocities", "estimate_state/imu_data", "estimate_state/robot_trajectory_point"},
        [this](std::array<Eigen::Vector3d, 4>** joint_positions_in, std::array<Eigen::Vector3d, 4>** joint_velocities_in, ImuData** imu_data_in, TrajectoryPoint** trajectory_point_out)
        {
            if (*joint_positions_in == NULL) POND_LOG_RETURN("detect_foot_contacts/joint_positions is NULL");
            if (*joint_velocities_in == NULL) POND_LOG_RETURN("detect_foot_contacts/joint_velocities is NULL");
            if (*imu_data_in == NULL) POND_LOG_RETURN("detect_foot_contacts/imu_data is NULL");
            *trajectory_point_out = &trajectory_point;

            double avg_leg_height = 0;

            for (uint32_t i = 0; i < 4; i++)
            {
                Eigen::Vector3d p[4] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

                Sophus::SO3d R[3];
                R[0] = Sophus::SO3d::rotX(joint_positions_in[0][0][i][0]);
                p[1] = p[0] + R[0] * cfg.legs[i].segments[0].offset;

                R[1] = R[0] * Sophus::SO3d::rotY(joint_positions_in[0][0][i][1]);
                p[2] = p[1] + R[1] * cfg.legs[i].segments[1].offset;

                R[2] = R[1] * Sophus::SO3d::rotY(joint_positions_in[0][0][i][2]);
                p[3] = p[2] + R[2] * cfg.legs[i].segments[2].offset;

                avg_leg_height += ((imu_data_in[0]->orientation * (p[3] + cfg.legs[i].origin_pos)).z() - cfg.legs[i].foot_radius)/ -4.0;
            }

            trajectory_point.stamp.time = imu_data_in[0]->stamp.time;
            trajectory_point.pose = Sophus::SE3d(imu_data_in[0]->orientation, Eigen::Vector3d(0,0,avg_leg_height));
            trajectory_point.lin_vel.setZero();
            trajectory_point.ang_vel = imu_data_in[0]->orientation * imu_data_in[0]->ang_vel;
        }
    );

    return POND_SUCCESS;
}

void ZeroStateEstimator::onShutdownTF()
{
    stage_callback.destroy();
}