#include "pond/hpp/dds.hpp"
#include "pond/hpp/module_base.hpp"
#include "pond_data_types/robot_state_types.hpp"
#include "pond_data_types/transform_types.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>
#include <pond_data_types/motor_types.hpp>
#include <pond_data_types/trajectory_types.hpp>
#include <sophus/se3.hpp>
#include <vector>

struct link_data
{
    double mass;
    Eigen::Vector3d com;
    Eigen::Matrix3d inertia;
    double joint_min, joint_max, effort_max;
};

struct leg_config
{
    double length_hip, length_thigh, length_shin;
    Eigen::Vector3d origin_pos;
    link_data links[3];
};

struct leg_dynamics
{
    Eigen::Vector3d p_foot, v_foot;
    Eigen::Matrix3d jacobian;
    Eigen::Matrix3d jacobian_dot;
    Eigen::Matrix3d lambda;
    Eigen::Vector3d c_qdot;
    Eigen::Vector3d g;
};

class LegTorqueController : public pond::ModuleBaseTF
{
public:
    virtual pond_result onStartupTF(const std::vector<void*>& args) override;
    virtual void onShutdownTF() override;
private:

    void calculate_leg_dynamics(uint32_t leg_i, leg_dynamics& dynamics, const Eigen::Vector3d& q, const Eigen::Vector3d& q_dot, const Eigen::Vector3d& gravity, bool calc_ff_torque);

    pond::Callback force_callback, stage_callback;
    pond::DistributorTyped<std::vector<double>> torque_distributor;
    pond::DistributorTyped<std::vector<FrameTransform>> poses_distributor;
    std::vector<double> distributed_torques;
    std::vector<FrameTransform> distributed_poses;
    std::vector<Eigen::Vector3d> forces_buf;
    std::mutex mutex;

    leg_config configs[4];

    double natural_frequency, max_swing_torque, oob_kp, inertia_damping_factor;

    std::vector<std::string> motor_names;
    std::vector<MotorCommand> motor_commands;
    std::vector<JointState> joint_states;
};

POND_MODULE_CPP_DECLARE(LegTorqueController, "leg_torque_controller", "torque control block")

pond_result LegTorqueController::onStartupTF(const std::vector<void*>& args)
{
    natural_frequency = parameter("natural_frequency").asDouble().get(20.0);
    max_swing_torque = parameter("max_swing_torque").asDouble().get(1.0);
    oob_kp = std::abs(parameter("oob_kp").asDouble().get(0.1));
    inertia_damping_factor = std::abs(parameter("inertia_damping_factor").asDouble().get(0.01));

    auto reference_link = parameter("reference_link").asString().get("base_link");

    forces_buf.resize(4, Eigen::Vector3d::Zero());
    distributed_torques.resize(12);
    distributed_poses.resize(4, {.stamp = {.frame_id = reference_link}});

    auto joint_names = parameter("joint_names").asStringArray().getStrict(12, 12); if (!joint_names) return POND_ERROR;
    auto motor_names_o = parameter("motor_names").asStringArray().getStrict(12, 12); if (!motor_names_o) return POND_ERROR;
    auto foot_link_names = parameter("foot_link_names").asStringArray().getStrict(4, 4); if (!foot_link_names) return POND_ERROR;
    
    motor_names = *motor_names_o;

    
    for (uint32_t l = 0; l < 4; l++)
    {
        GetJointInfoRequest joint_requests[3];
        GetFrameTransformRequest tf_requests[4];

        for (uint32_t i = 0; i < 3; i++)
        {
            if (!tfGetJointInfo(joint_names->at(3*l+i), joint_requests[i])) return POND_ERROR;
            if (!joint_requests[i].min_angle || !joint_requests[i].max_angle || !joint_requests[i].max_effort) POND_LOG_RETURN_ERROR("No joint position limits set for %s", joint_requests[i].joint_name.c_str());
        
            configs[l].links[i].joint_min = *joint_requests[i].min_angle;
            configs[l].links[i].joint_max = *joint_requests[i].max_angle;
            configs[l].links[i].effort_max = *joint_requests[i].max_effort;
        }

        if (!tfGetTransform(joint_requests[1].parent_link_name, joint_requests[0].child_link_name, 0, tf_requests[0])) return POND_ERROR;
        if (!tfGetTransform(joint_requests[2].parent_link_name, joint_requests[1].child_link_name, 0, tf_requests[1])) return POND_ERROR;
        if (!tfGetTransform(foot_link_names->at(l), joint_requests[2].child_link_name, 0, tf_requests[2])) return POND_ERROR;

        configs[l].length_hip   = (tf_requests[0].tf * joint_requests[1].tf).translation().norm();
        configs[l].length_thigh = (tf_requests[1].tf * joint_requests[2].tf).translation().norm();
        configs[l].length_shin  = (tf_requests[2].tf).translation().norm();

        if (!tfGetTransform(joint_requests[0].parent_link_name, reference_link, 0, tf_requests[3])) return POND_ERROR;
        configs[l].origin_pos = tf_requests[3].tf * joint_requests[0].tf * Eigen::Vector3d::Zero();

        GetLinkInfoRequest link_requests[3];
        for (uint32_t i = 0; i < 3; i++)
        {
            if (!tfGetLinkInfo(joint_requests[i].child_link_name, link_requests[i])) return POND_ERROR;
            configs[l].links[i].mass = link_requests[i].mass;
            configs[l].links[i].com = link_requests[i].inertia_tf.translation();
            configs[l].links[i].inertia = link_requests[i].inertia;
        }
    }

    torque_distributor = createDistributorTyped<std::vector<double>>({"target_torques"});
    poses_distributor = createDistributorTyped<std::vector<FrameTransform>>({"foot_poses"});

    joint_states.resize(12); motor_commands.resize(12);

    pond::ChannelsInfo info;
    for (uint32_t i = 0; i < 12; i++)
    {
        joint_states[i].joint_name = joint_names->at(i);
        motor_names[i] = motor_names_o->at(i);
        info.channel<MotorCommand*>("calculate_torque/" + motor_names[i] + "_command");
    }

    for (uint32_t i = 0; i < 12; i++) info.channel<MotorFeedback*>("calculate_torque/" + motor_names[i] + "_feedback");

    info.channel<std::vector<bool>*>("calculate_torque/gait_phases");
    info.channel<std::vector<TrajectoryState>*>("calculate_torque/target_trajectory_states");

    stage_callback = createCallback<void>(info, [this](void** data) {

        MotorCommand*** commands = (MotorCommand***)data;
        MotorFeedback*** feedbacks = (MotorFeedback***)&data[12];
        std::vector<bool>** gait_phases = (std::vector<bool>**)data[24];
        std::vector<TrajectoryState>** ref_states = (std::vector<TrajectoryState>**)data[25];

        if (*gait_phases == NULL) POND_LOG_RETURN("calculate_torque/gait_phases is NULL");
        if (*ref_states == NULL) POND_LOG_RETURN("calculate_torque/target_trajectory_states is NULL");
        for (uint32_t i = 0; i < 12; i++) if (*feedbacks[i] == NULL) POND_LOG_RETURN("%s is NULL", ("calculate_torque/" + motor_names[i] + "_feedback").c_str());
        for (uint32_t i = 0; i < 12; i++) if (!feedbacks[i][0]->pos) POND_LOG_RETURN("%s does not provide position feedback", motor_names[i].c_str());
        for (uint32_t i = 0; i < 12; i++) if (!feedbacks[i][0]->vel) POND_LOG_RETURN("%s does not provide velocity feedback", motor_names[i].c_str());
        for (uint32_t i = 0; i < 12; i++) *commands[i] = &motor_commands[i];
        
        mutex.lock();
        std::vector<Eigen::Vector3d> forces = forces_buf;
        mutex.unlock();

        for (uint32_t i = 0; i < 4; i++)
        {
            Eigen::Vector3d joint_pos = {*feedbacks[3*i+0][0]->pos, *feedbacks[3*i+1][0]->pos, *feedbacks[3*i+2][0]->pos};
            Eigen::Vector3d joint_vel = {*feedbacks[3*i+0][0]->vel, *feedbacks[3*i+1][0]->vel, *feedbacks[3*i+2][0]->vel};
            Eigen::Vector3d torque = Eigen::Vector3d::Zero(), oob_torque = Eigen::Vector3d::Zero();
            
            leg_dynamics dynamics;

            std::array<bool, 3> oob = {false, false, false};

            for (uint32_t j = 0; j < 3; j++)
            {
                if (configs[i].links[j].joint_min > joint_pos[j])
                {
                    oob[j] = true;
                    oob_torque[j] = oob_kp * (configs[i].links[j].joint_min - joint_pos[j]);
                    joint_pos[j] = configs[i].links[j].joint_min;
                }
                else if (configs[i].links[j].joint_max < joint_pos[j])
                {
                    oob[j] = true;
                    oob_torque[j] = oob_kp * (configs[i].links[j].joint_max - joint_pos[j]);
                    joint_pos[j] = configs[i].links[j].joint_max;
                }
            }


            if (!gait_phases[0][0][i])
            {
                calculate_leg_dynamics(i, dynamics, joint_pos, joint_vel, Eigen::Vector3d(0, 0, -9.81), true);

                Eigen::Matrix3d Kp = Eigen::Matrix3d::Zero(), Kd = Eigen::Matrix3d::Zero();

                for (uint32_t i = 0; i < 3; i++)
                {
                    Kp(i, i) = natural_frequency * natural_frequency * dynamics.lambda(i, i);
                    Kd(i, i) = 2.0 * natural_frequency * dynamics.lambda(i, i);
                }

                Eigen::Vector3d pos_error = ref_states[0][0][i].pos - dynamics.p_foot, vel_error = ref_states[0][0][i].vel - dynamics.v_foot;

                torque = dynamics.jacobian.transpose() * (
                    Kp * pos_error + Kd * vel_error +
                    dynamics.lambda * (ref_states[0][0][i].acc - dynamics.jacobian_dot * joint_vel)
                ) + dynamics.c_qdot + dynamics.g;

                for (uint32_t j = 0; j < 3; j++) torque[j] = std::clamp(torque[j], -max_swing_torque, max_swing_torque);
            }
            else
            {
                calculate_leg_dynamics(i, dynamics, joint_pos, joint_vel, Eigen::Vector3d(0, 0, -9.81), false);
                torque = dynamics.jacobian.transpose() * forces[i];
            }
            
            for (uint32_t j = 0; j < 3; j++)
            {
                if (oob[j]) torque[j] = oob_torque[j];
                else if (!std::isfinite(torque[j])) torque[j] = 0;
                else torque[j] = std::clamp(torque[j], -configs[i].links[j].effort_max, configs[i].links[j].effort_max);
            }

            motor_commands[3*i+0].torque = torque[0];
            motor_commands[3*i+1].torque = torque[1];
            motor_commands[3*i+2].torque = torque[2];

            distributed_torques[3*i+0] = torque[0];
            distributed_torques[3*i+1] = torque[1];
            distributed_torques[3*i+2] = torque[2];

            double vel_angle = (std::abs(dynamics.v_foot[0]) > 0.01 || std::abs(dynamics.v_foot[1]) > 0.01 ? atan2(dynamics.v_foot[1] , dynamics.v_foot[0]) : 0);

            distributed_poses[i].stamp.time = pond::get_time();
            distributed_poses[i].tf = Sophus::SE3d(Sophus::SO3d::rotZ(vel_angle), dynamics.p_foot);
        }

        torque_distributor.distribute(&distributed_torques);
        poses_distributor.distribute(&distributed_poses);
    });

    force_callback = createCallback<std::vector<Eigen::Vector3d>>({"ground_forces"}, [this](std::vector<Eigen::Vector3d>* forces_in) {
        std::lock_guard<std::mutex> lock(mutex);
        forces_buf = *forces_in;
    });

    return POND_SUCCESS;
}

void LegTorqueController::onShutdownTF()
{
    stage_callback.destroy();
    force_callback.destroy();
    torque_distributor.destroy();
    poses_distributor.destroy();
}

void LegTorqueController::calculate_leg_dynamics(uint32_t leg_i, leg_dynamics& dynamics, const Eigen::Vector3d& q, const Eigen::Vector3d& q_dot, const Eigen::Vector3d& gravity, bool calc_ff_torque)
{
    // 1. Forward Kinematics & Transforms
    Sophus::SO3d R[3];
    Eigen::Vector3d p[4] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

    // Apply side_sign to the lateral hip offset vector
    R[0] = Sophus::SO3d::rotX(q[0]);
    p[1] = p[0] + R[0] * Eigen::Vector3d(0.0, (leg_i%2 ? -1.0 : 1.0) * configs[leg_i].length_hip, 0.0);

    R[1] = R[0] * Sophus::SO3d::rotY(q[1]);
    p[2] = p[1] + R[1] * Eigen::Vector3d(0.0, 0.0, -configs[leg_i].length_thigh);

    R[2] = R[1] * Sophus::SO3d::rotY(q[2]);
    p[3] = p[2] + R[2] * Eigen::Vector3d(0.0, 0.0, -configs[leg_i].length_shin);

    dynamics.p_foot = p[3] + configs[leg_i].origin_pos;

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

    if (!calc_ff_torque) return;

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
    Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
    dynamics.g = Eigen::Vector3d::Zero();
    dynamics.c_qdot = Eigen::Vector3d::Zero();

    Eigen::Vector3d a_prev = Eigen::Vector3d::Zero();

    for (uint32_t k = 0; k < 3; k++)
    {
        // CoM and Inertia in base frame (parsed directly from URDF)
        Eigen::Vector3d p_com_k = p[k] + R[k] * configs[leg_i].links[k].com;
        Eigen::Matrix3d I_base_k = R[k].matrix() * configs[leg_i].links[k].inertia * R[k].matrix().transpose();

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
        M += configs[leg_i].links[k].mass * Jv_com.transpose() * Jv_com + Jw_com.transpose() * I_base_k * Jw_com;
        dynamics.g -= Jv_com.transpose() * (configs[leg_i].links[k].mass * gravity);

        // Accumulate Coriolis Term (C * q_dot) via Recursive Newton-Euler mapping
        Eigen::Vector3d gyro = omega[k].cross(I_base_k * omega[k]);
        dynamics.c_qdot += Jv_com.transpose() * (configs[leg_i].links[k].mass * a_com_k) + Jw_com.transpose() * gyro;
    }

    // 6. Compute Operational Space Inertia Matrix ($\boldsymbol{\Lambda}$)
    
    Eigen::Matrix3d lambda_inverse = dynamics.jacobian * M.inverse() * dynamics.jacobian.transpose();
    
    lambda_inverse += (inertia_damping_factor * inertia_damping_factor) * Eigen::Matrix3d::Identity();

    dynamics.lambda = lambda_inverse.inverse();
}