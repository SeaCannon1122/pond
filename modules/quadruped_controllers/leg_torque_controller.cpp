#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>
#include <pond_data_types/motor_types.hpp>
#include <pond_data_types/trajectory_types.hpp>

#include "quadruped_robot_config.hpp"

struct leg_dynamics
{
    Eigen::Vector3d p_foot, v_foot;

    Eigen::Matrix3d jacobian;
    Eigen::Matrix3d jacobian_dot;

    Eigen::Matrix3d m, lambda;
    Eigen::Vector3d c_qdot, g;
};

class LegTorqueController : public pond::ModuleBaseTF
{
public:
    virtual pond_result onStartupTF(const std::vector<void*>& args) override;
    virtual void onShutdownTF() override;
private:
    bool calculate_leg_dynamics(uint32_t leg_i, leg_dynamics& dynamics, const Eigen::Vector3d& q, const Eigen::Vector3d& q_dot, const Eigen::Vector3d& gravity);

    QuadrupedRobotConfig cfg;
    pond::Callback stage_callback;

    double natural_frequency, oob_spring_frequency, max_swing_torque, determinant_threshold;

    std::array<Eigen::Vector3d, 4> last_oob_free_joint_positions, target_torques;
};

POND_MODULE_CPP_DECLARE(LegTorqueController, "leg_torque_controller", "torque control block")

pond_result LegTorqueController::onStartupTF(const std::vector<void*>& args)
{
    determinant_threshold = parameter("determinant_threshold").asDouble().get(1e-9);
    natural_frequency = parameter("natural_frequency").asDouble().get(180.0);
    oob_spring_frequency = parameter("oob_spring_frequency").asDouble().get(2.0);
    max_swing_torque = parameter("max_swing_torque").asDouble().get(1.0);

    if (!cfg.init(*this)) return POND_ERROR;
    for (uint32_t l = 0; l < 4; l++) last_oob_free_joint_positions[l] = Eigen::Vector3d(0, 0, cfg.legs[l].segments[2].joint_max); 

    stage_callback = createCallback<
        std::vector<bool>*, std::vector<TrajectoryPoint>*, std::vector<Eigen::Vector3d>*,
        std::array<Eigen::Vector3d, 4>*, std::array<Eigen::Vector3d, 4>*, std::array<Eigen::Vector3d, 4>*
    >({
        "calculate_torque/gait_phases", "calculate_torque/target_swing_trajectory_points", "calculate_torque/ground_forces",
        "calculate_torque/joint_positions", "calculate_torque/joint_velocities", "calculate_torque/joint_target_torques"
    }, [this](
        std::vector<bool>** gait_phases, std::vector<TrajectoryPoint>** ref_states, std::vector<Eigen::Vector3d>** ground_forces_in,
        std::array<Eigen::Vector3d, 4>** joint_positions_in, std::array<Eigen::Vector3d, 4>** joint_velocities_in, std::array<Eigen::Vector3d, 4>** joint_target_torques_out) {

        if (*gait_phases == NULL)         POND_LOG_RETURN("calculate_torque/gait_phases is NULL");
        if (*ref_states == NULL)          POND_LOG_RETURN("calculate_torque/target_swing_trajectory_points is NULL");
        if (*ground_forces_in == NULL)    POND_LOG_RETURN("calculate_torque/ground_forces is NULL");
        if (*joint_positions_in == NULL)  POND_LOG_RETURN("calculate_torque/joint_positions is NULL");
        if (*joint_velocities_in == NULL) POND_LOG_RETURN("calculate_torque/joint_velocities is NULL");

        *joint_target_torques_out = &target_torques;

        for (uint32_t i = 0; i < 4; i++)
        {
            Eigen::Vector3d joint_pos_actual = joint_positions_in[0][0][i], joint_vel = joint_velocities_in[0][0][i];
            Eigen::Vector3d joint_pos = joint_pos_actual, torque = Eigen::Vector3d::Zero();

            leg_dynamics dynamics, dynamics_actual;

            uint32_t oob = 0;
            // if (!calculate_leg_dynamics(i, dynamics_actual, joint_pos_actual, joint_vel, Eigen::Vector3d(0, 0, -9.81)))
            // {
            //     for (uint32_t j = 0; j < 3; j++)
            //     {
            //         if (configs[i].links[j].joint_min > joint_pos[j])
            //         {
            //             oob |= 1<<j;
            //             joint_pos[j] = configs[i].links[j].joint_min;
            //         }
            //         else if (configs[i].links[j].joint_max < joint_pos[j])
            //         {
            //             oob |= 1<<j;
            //             joint_pos[j] = configs[i].links[j].joint_max;
            //         }
            //     }

            //     if (gait_phases[0][0][i]) for (uint32_t j = 0; j < 3; j++)
            //     {
            //         joint_pos[j] = last_oob_free_joint_positions[i][j];
            //         oob |= 1<<j;
            //     }

            // } else last_oob_free_joint_positions[i] = joint_pos;

            calculate_leg_dynamics(i, dynamics, joint_pos, joint_vel, Eigen::Vector3d(0, 0, -9.81));
            
            if (!gait_phases[0][0][i])
            {
                Eigen::Matrix3d Kp = Eigen::Matrix3d::Zero(), Kd = Eigen::Matrix3d::Zero();

                for (uint32_t i = 0; i < 3; i++)
                {
                    Kp(i, i) = natural_frequency * natural_frequency * dynamics.lambda(i, i);
                    Kd(i, i) = 2.0 * natural_frequency * dynamics.lambda(i, i);
                }

                Eigen::Vector3d pos_error = ref_states[0][0][i].pose.translation() - dynamics.p_foot, vel_error = ref_states[0][0][i].lin_vel - dynamics.v_foot;

                torque = dynamics.jacobian.transpose() * (
                    Kp * pos_error + Kd * vel_error +
                    dynamics.lambda * (ref_states[0][0][i].lin_acc - dynamics.jacobian_dot * joint_vel)
                ) + dynamics.c_qdot + dynamics.g;

                for (uint32_t j = 0; j < 3; j++) torque[j] = std::clamp(torque[j], -max_swing_torque, max_swing_torque);
            }
            else if (oob == 0) torque = -dynamics.jacobian.transpose() * ground_forces_in[0][0][i];
            
            for (uint32_t j = 0; j < 3; j++)
            {
                if (oob & (1<<j)) torque[j] =
                    oob_spring_frequency * oob_spring_frequency * dynamics_actual.m(j, j) * (joint_pos[j] - joint_pos_actual[j])
                    - 2.0 * oob_spring_frequency * dynamics_actual.m(j, j) * joint_vel[j]
                    + dynamics_actual.c_qdot[j] + dynamics_actual.g[j];
                
                if (std::isnan(torque[j])) torque[j] = 0;
                else torque[j] = std::clamp(torque[j], -cfg.legs[i].segments[j].effort_max, cfg.legs[i].segments[j].effort_max);
            }

            target_torques[i] = torque;
        }
    });

    return POND_SUCCESS;
}

void LegTorqueController::onShutdownTF() { stage_callback.destroy(); }

bool LegTorqueController::calculate_leg_dynamics(uint32_t leg_i, leg_dynamics& dynamics, const Eigen::Vector3d& q, const Eigen::Vector3d& q_dot, const Eigen::Vector3d& gravity)
{
    // 1. Forward Kinematics & Transforms
    Sophus::SO3d R[3];
    Eigen::Vector3d p[4] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

    R[0] = Sophus::SO3d::rotX(q[0]);
    p[1] = p[0] + R[0] * cfg.legs[leg_i].segments[0].offset;

    R[1] = R[0] * Sophus::SO3d::rotY(q[1]);
    p[2] = p[1] + R[1] * cfg.legs[leg_i].segments[1].offset;

    R[2] = R[1] * Sophus::SO3d::rotY(q[2]);
    p[3] = p[2] + R[2] * cfg.legs[leg_i].segments[2].offset;

    dynamics.p_foot = p[3] + cfg.legs[leg_i].origin_pos;

    Eigen::Vector3d w[3] = {
               Eigen::Vector3d::UnitX(),
        R[0] * Eigen::Vector3d::UnitY(),
        R[1] * Eigen::Vector3d::UnitY()
    };

    // 2. Compute Foot Jacobian (J)
    dynamics.jacobian.col(0) = w[0].cross(p[3] - p[0]);
    dynamics.jacobian.col(1) = w[1].cross(p[3] - p[1]);
    dynamics.jacobian.col(2) = w[2].cross(p[3] - p[2]);

    dynamics.v_foot = dynamics.jacobian * q_dot;

    // 3. Angular & Linear Velocities (Required for J_dot and Coriolis)
    Eigen::Vector3d omega[3];
    omega[0] =            w[0] * q_dot[0];
    omega[1] = omega[0] + w[1] * q_dot[1];
    omega[2] = omega[1] + w[2] * q_dot[2];

    Eigen::Vector3d w_dot[3];
    w_dot[0] = Eigen::Vector3d::Zero();
    w_dot[1] = omega[0].cross(w[1]);
    w_dot[2] = omega[1].cross(w[2]);

    Eigen::Vector3d p_dot[4];
    p_dot[0] = Eigen::Vector3d::Zero();
    p_dot[1] = p_dot[0] + omega[0].cross(p[1] - p[0]);
    p_dot[2] = p_dot[1] + omega[1].cross(p[2] - p[1]);
    p_dot[3] = p_dot[2] + omega[2].cross(p[3] - p[2]);

    // 4. Compute Foot Jacobian Derivative (J_dot)
    for (uint32_t i = 0; i < 3; i++)
    {
        Eigen::Vector3d r_i = p[3] - p[i];
        Eigen::Vector3d r_dot_i = p_dot[3] - p_dot[i];
        dynamics.jacobian_dot.col(i) = w_dot[i].cross(r_i) + w[i].cross(r_dot_i);
    }

    // 5. Compute Mass Matrix (M), Gravity (G), and Coriolis (C*q_dot)
    dynamics.m = Eigen::Matrix3d::Zero();
    dynamics.g = Eigen::Vector3d::Zero();
    dynamics.c_qdot = Eigen::Vector3d::Zero();

    Eigen::Vector3d a_prev = Eigen::Vector3d::Zero();

    for (uint32_t k = 0; k < 3; k++)
    {
        // CoM and Inertia in base frame (parsed directly from URDF)
        Eigen::Vector3d p_com_k = p[k] + R[k] * cfg.legs[leg_i].segments[k].com;
        Eigen::Matrix3d I_base_k = R[k].matrix() * cfg.legs[leg_i].segments[k].inertia * R[k].matrix().transpose();

        // Link origin acceleration & CoM acceleration
        Eigen::Vector3d r_link = p[k+1] - p[k];
        Eigen::Vector3d a_link = a_prev + omega[k].cross(omega[k].cross(r_link));
        a_prev = a_link;
        Eigen::Vector3d a_com_k = a_link + omega[k].cross(omega[k].cross(p_com_k - p[k]));

        // CoM Jacobians
        Eigen::Matrix3d Jv_com = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d Jw_com = Eigen::Matrix3d::Zero();
        for (uint32_t i = 0; i <= k; i++)
        {
            Jv_com.col(i) = w[i].cross(p_com_k - p[i]);
            Jw_com.col(i) = w[i];
        }

        // Accumulate Mass Matrix & Gravity Vector
        dynamics.m += cfg.legs[leg_i].segments[k].mass * Jv_com.transpose() * Jv_com + Jw_com.transpose() * I_base_k * Jw_com;
        dynamics.g -= Jv_com.transpose() * (cfg.legs[leg_i].segments[k].mass * gravity);

        // Accumulate Coriolis Term (C * q_dot) via Recursive Newton-Euler mapping
        Eigen::Vector3d gyro = omega[k].cross(I_base_k * omega[k]);
        dynamics.c_qdot += Jv_com.transpose() * (cfg.legs[leg_i].segments[k].mass * a_com_k) + Jw_com.transpose() * gyro;
    }

    // 6. Compute Operational Space Inertia Matrix ($\boldsymbol{\Lambda}$)
    
    Eigen::Matrix3d lambda_inverse = dynamics.jacobian * dynamics.m.inverse() * dynamics.jacobian.transpose();

    dynamics.lambda = lambda_inverse.inverse();

    return (dynamics.jacobian.determinant() > determinant_threshold);
}