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

class StateEstimator : public pond::ModuleBaseTF
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

    // work data
    Vector18 x_hat;

    Vector12 ps, vs;
    Matrix18x18 A, P;
    Matrix18x3 B;
    Matrix28x18 C;
};

POND_MODULE_CPP_DECLARE(StateEstimator, "state_estimator", "estimates robot state")

pond_result StateEstimator::onStartupTF(const std::vector<void*>& args)
{
    if (!cfg.init(*this)) return POND_ERROR;

    noise.sensor.foot_position  = parameter("noise.sensor.foot_position") .asDouble().get(0.001);
    noise.sensor.foot_velocity  = parameter("noise.sensor.foot_velocity") .asDouble().get(0.1);
    noise.sensor.foot_height    = parameter("noise.sensor.foot_height")   .asDouble().get(0.001);
    noise.process.foot_position = parameter("noise.process.foot_position").asDouble().get(0.002);
    noise.process.imu_position  = parameter("noise.process.imu_position") .asDouble().get(0.02);
    noise.process.imu_velocity  = parameter("noise.process.imu_velocity") .asDouble().get(0.02);

    frame_dt = parameter("frame_dt").asDouble().get(0.001);

    x_hat.setZero();
    x_hat[2] = 0.18;

    ps.setZero(); vs.setZero();
    A.setIdentity();
    A.block<3,3>(0, 3).diagonal().setConstant(frame_dt);

    B.setZero();
    B.block<3,3>(3, 0).diagonal().setConstant(frame_dt);

    C.setZero();

    for (uint32_t i = 0; i < 4; i++)
    {
        C.block<3, 3>(3*i, 0).setIdentity();
        C.block<3, 3>(3*i+12, 3).setIdentity();
    }

    C.block<12, 12>(0, 6) = -Matrix12x12::Identity();

    for (uint32_t i = 0; i < 4; i++) C(27-i, 17-3*i) = 1.0;
    
    P = Matrix18x18::Identity() * 100.0;

    stage_callback = createCallback<std::array<Eigen::Vector3d, 4>*, std::array<Eigen::Vector3d, 4>*, ImuData*, TrajectoryPoint*>(
        {"estimate_state/joint_positions", "estimate_state/joint_velocities", "estimate_state/imu_data", "estimate_state/robot_trajectory_point"},
        [this](std::array<Eigen::Vector3d, 4>** joint_positions_in, std::array<Eigen::Vector3d, 4>** joint_velocities_in, ImuData** imu_data_in, TrajectoryPoint** trajectory_point_out)
        {
            if (*joint_positions_in == NULL) POND_LOG_RETURN("estimate_state/joint_positions is NULL");
            if (*joint_velocities_in == NULL) POND_LOG_RETURN("estimate_state/joint_velocities is NULL");
            if (*imu_data_in == NULL) POND_LOG_RETURN("estimate_state/imu_data is NULL");
            *trajectory_point_out = &trajectory_point;

            Matrix18x18 Q = Matrix18x18::Identity();
            Q.block< 3, 3>(0, 0).diagonal() *= frame_dt        / 20.f * noise.process.imu_position;
            Q.block< 3, 3>(3, 3).diagonal() *= frame_dt * 9.8f / 20.f * noise.process.imu_velocity;
            Q.block<12,12>(6, 6).diagonal() *= frame_dt *               noise.process.foot_position;

            Matrix28x28 R = Matrix28x28::Identity();
            R.block<12,12>( 0,  0).diagonal() *= noise.sensor.foot_position;
            R.block<12,12>(12, 12).diagonal() *= noise.sensor.foot_velocity;
            R.block< 4, 4>(24, 24).diagonal() *= noise.sensor.foot_height;

            Eigen::Matrix3d Rbod = imu_data_in[0]->orientation.matrix().transpose();

            Eigen::Vector3d a = imu_data_in[0]->orientation * imu_data_in[0]->lin_acc; 

            Eigen::Vector4d pzs = Eigen::Vector4d::Zero();
            Eigen::Vector4d trusts = Eigen::Vector4d::Zero();

            Eigen::Vector3d p0 = x_hat.segment<3>(0), v0 = x_hat.segment<3>(3);

            for (uint32_t i = 0, i1 = 0; i < 4; i++, i1 = 3*i)
            {
                Eigen::Matrix3d jacobian;
                Eigen::Vector3d p[4] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
                {
                    Sophus::SO3d R[3];
                    R[0] = Sophus::SO3d::rotX(joint_positions_in[0][0][i][0]);
                    p[1] = p[0] + R[0] * cfg.legs[i].segments[0].offset;

                    R[1] = R[0] * Sophus::SO3d::rotY(joint_positions_in[0][0][i][1]);
                    p[2] = p[1] + R[1] * cfg.legs[i].segments[1].offset;

                    R[2] = R[1] * Sophus::SO3d::rotY(joint_positions_in[0][0][i][2]);
                    p[3] = p[2] + R[2] * cfg.legs[i].segments[2].offset;

                    jacobian.col(0) = (       Eigen::Vector3d::UnitX()).cross(p[3] - p[0]);
                    jacobian.col(1) = (R[0] * Eigen::Vector3d::UnitY()).cross(p[3] - p[1]);
                    jacobian.col(2) = (R[1] * Eigen::Vector3d::UnitY()).cross(p[3] - p[2]);
                }

                Eigen::Vector3d p_foot = p[3] + cfg.legs[i].origin_pos - Eigen::Vector3d(0, 0, cfg.legs[i].foot_radius);
                Eigen::Vector3d v_foot = jacobian * joint_velocities_in[0][0][i];

                Eigen::Vector3d p_f = Rbod * p_foot;
                Eigen::Vector3d dp_f = Rbod * (imu_data_in[0]->ang_vel.cross(p_foot) + v_foot);

                double trust = 1.0, trust_window = 0.2;
                double phase = fmin(/*MISSING contact estimate*/ 0.5, 1.0);

                if      (phase <        trust_window ) trust =        phase  / trust_window;
                else if (phase > (1.0 - trust_window)) trust = (1.0 - phase) / trust_window;
                
                double high_suspect_number = 100;

                Q.block<3,3>( 6+i1,  6+i1) *= (1.0 + (1.0 - trust) * high_suspect_number);
                R.block<3,3>(   i1,    i1) *= 1.0;
                R.block<3,3>(12+i1, 12+i1) *= (1.0 + (1.0 - trust) * high_suspect_number);
                R           (     24+ i,      24+ i) *= (1.0 + (1.0 - trust) * high_suspect_number);

                trusts(i) = trust;

                ps.segment<3>(i1) = -p_f;
                vs.segment<3>(i1) = (1.0f - trust) * v0 + trust * (-dp_f);
                pzs(i) = (1.0f - trust) * (p0(2) + p_f(2));
            }

            x_hat = A * x_hat + B * a;

            Matrix18x18 Pm = A * P * A.transpose() + Q;
            Matrix28x28 S = C * Pm * C.transpose() + R;
            
            Vector28 ey = (Vector28() << ps, vs, pzs).finished() - C * x_hat;

            x_hat += Pm * C.transpose() * S.lu().solve(ey);

            P = (Matrix18x18::Identity() - Pm * C.transpose() * S.lu().solve(C)) * Pm;
            P = (P + P.transpose()) / 2.0;

            if (P.block<2,2>(0, 0).determinant() > 1e-6)
            {
                P.block<2,16>(0, 2).setZero();
                P.block<16,2>(2, 0).setZero();
                P.block<2,2>(0, 0) /= 10.0;
            }

            trajectory_point.stamp.time = imu_data_in[0]->stamp.time;
            trajectory_point.pose = Sophus::SE3d(imu_data_in[0]->orientation, x_hat.segment<3>(0));
            trajectory_point.lin_vel = x_hat.segment<3>(3);
        }
    );

    return POND_SUCCESS;
}

void StateEstimator::onShutdownTF() { stage_callback.destroy();}