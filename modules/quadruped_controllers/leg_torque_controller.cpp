#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>
#include <pond_data_types/motor_types.hpp>
#include <pond_data_types/trajectory_types.hpp>

struct link_data
{
    double mass;
    Eigen::Vector3d com;
    Eigen::Matrix3d inertia;
};

struct leg_config
{
    double length_hip, length_thigh, length_shin;
    link_data links[3];
};

struct leg_dynamics
{
    Eigen::Vector3d p_foot;
    Eigen::Matrix3d jacobian;
    Eigen::Matrix3d jacobian_dot;
    Eigen::Matrix3d lambda;
    Eigen::Vector3d c_qdot;
    Eigen::Vector3d g;
};

void calculate_leg_dynamics(
    leg_dynamics& dynamics,
    const leg_config& config,
    const Eigen::Vector3d& q,
    const Eigen::Vector3d& q_dot,
    const Eigen::Vector3d& gravity,
    bool calc_ff_torque,
    double side_sign
)
{
    // 1. Forward Kinematics & Transforms
    Sophus::SO3d R[3];
    Eigen::Vector3d p[4] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

    // Apply side_sign to the lateral hip offset vector
    R[0] = Sophus::SO3d::rotX(q[0]);
    p[1] = p[0] + R[0] * Eigen::Vector3d(0.0, side_sign * config.length_hip, 0.0);

    R[1] = R[0] * Sophus::SO3d::rotY(q[1]);
    p[2] = p[1] + R[1] * Eigen::Vector3d(0.0, 0.0, -config.length_thigh);

    R[2] = R[1] * Sophus::SO3d::rotY(q[2]);
    p[3] = p[2] + R[2] * Eigen::Vector3d(0.0, 0.0, -config.length_shin);

    dynamics.p_foot = p[3];

    Eigen::Vector3d w[3] = {
               Eigen::Vector3d::UnitX(),
        R[0] * Eigen::Vector3d::UnitY(),
        R[1] * Eigen::Vector3d::UnitY()
    };

    // 2. Compute Foot Jacobian (J)
    dynamics.jacobian.col(0) = w[0].cross(p[3] - p[0]);
    dynamics.jacobian.col(1) = w[1].cross(p[3] - p[1]);
    dynamics.jacobian.col(2) = w[2].cross(p[3] - p[2]);

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
        Eigen::Vector3d p_com_k = p[k] + R[k] * config.links[k].com;
        Eigen::Matrix3d I_base_k = R[k].matrix() * config.links[k].inertia * R[k].matrix().transpose();

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
        M += config.links[k].mass * Jv_com.transpose() * Jv_com + Jw_com.transpose() * I_base_k * Jw_com;
        dynamics.g -= Jv_com.transpose() * (config.links[k].mass * gravity);

        // Accumulate Coriolis Term (C * q_dot) via Recursive Newton-Euler mapping
        Eigen::Vector3d gyro = omega[k].cross(I_base_k * omega[k]);
        dynamics.c_qdot += Jv_com.transpose() * (config.links[k].mass * a_com_k) + Jw_com.transpose() * gyro;
    }

    // 6. Compute Operational Space Inertia Matrix ($\boldsymbol{\Lambda}$)
    Eigen::Matrix3d J_inv = dynamics.jacobian.inverse();
    dynamics.lambda = J_inv.transpose() * M * J_inv;
}

class LegTorqueController : public pond::ModuleBaseTF
{
public:
    virtual pond_result onStartupTF(const std::vector<void*>& args) override;
    virtual void onShutdownTF() override;
private:
    pond::Callback force_callback, mode_callback, stage_callback, trajectory_states_callback;
    std::vector<bool> modes_buf;
    std::vector<Eigen::Vector3d> forces_buf;
    std::vector<TrajectoryState> ref_trajectory_states_buf;
    std::mutex mutex;

    leg_config configs[4];

    double natural_frequency;

    std::vector<std::string> motor_names;
    std::vector<MotorCommand> motor_commands;
    std::vector<JointState> joint_states;
};

POND_MODULE_CPP_DECLARE(LegTorqueController, "leg_torque_controller", "torque control block")

pond_result LegTorqueController::onStartupTF(const std::vector<void*>& args)
{
    natural_frequency = parameter("natural_frequency").asDouble().get(20.0);

    forces_buf.resize(4, Eigen::Vector3d::Zero());
    modes_buf.resize(4, false);

    auto joint_names = parameter("joint_names").asStringArray().getStrict(12, 12); if (!joint_names) return POND_ERROR;
    auto motor_names_o = parameter("motor_names").asStringArray().getStrict(12, 12); if (!motor_names_o) return POND_ERROR;
    auto foot_link_names = parameter("foot_link_names").asStringArray().getStrict(4, 4); if (!foot_link_names) return POND_ERROR;
    
    motor_names = *motor_names_o;

    for (uint32_t l = 0; l < 4; l++)
    {
        GetJointInfoRequest joint_requests[3];
        GetFrameTransformRequest tf_requests[4];

        for (uint32_t i = 0; i < 3; i++) if (!tfGetJointInfo(joint_names->at(3*l+i), joint_requests[i])) return POND_ERROR;
        
        if (!tfGetTransform(joint_requests[1].parent_link_name, joint_requests[0].child_link_name, 0, tf_requests[0])) return POND_ERROR;
        if (!tfGetTransform(joint_requests[2].parent_link_name, joint_requests[1].child_link_name, 0, tf_requests[1])) return POND_ERROR;
        if (!tfGetTransform(foot_link_names->at(l), joint_requests[2].child_link_name, 0, tf_requests[2])) return POND_ERROR;

        configs[l].length_hip   = (tf_requests[0].tf * joint_requests[1].tf).translation().norm();
        configs[l].length_thigh = (tf_requests[1].tf * joint_requests[2].tf).translation().norm();
        configs[l].length_shin  = (tf_requests[2].tf).translation().norm();

        GetLinkInfoRequest link_requests[3];
        for (uint32_t i = 0; i < 3; i++)
        {
            if (!tfGetLinkInfo(joint_requests[i].child_link_name, link_requests[i])) return POND_ERROR;
            configs[l].links[i].mass = link_requests[i].mass;
            configs[l].links[i].com = link_requests[i].inertia_tf.translation();
            configs[l].links[i].inertia = link_requests[i].inertia;
        }
    }

    joint_states.resize(12); motor_commands.resize(12);

    pond::ChannelsInfo info;
    for (uint32_t i = 0; i < 12; i++)
    {
        joint_states[i].joint_name = joint_names->at(i);
        motor_names[i] = motor_names_o->at(i);
        info.channel<MotorCommand*>("calculate_torque/" + motor_names[i] + "_command");
    }

    for (uint32_t i = 0; i < 12; i++) info.channel<MotorFeedback*>("calculate_torque/" + motor_names[i] + "_feedback");

    stage_callback = createCallback<void>(info, [this](void** data) {

        MotorCommand*** commands = (MotorCommand***)data;
        MotorFeedback*** feedbacks = (MotorFeedback***)&data[12];

        for (uint32_t i = 0; i < 12; i++) if (*feedbacks[i] == NULL) POND_LOG_RETURN("%s is NULL", ("calculate_torque/" + motor_names[i] + "_feedback").c_str());
        for (uint32_t i = 0; i < 12; i++) if (!feedbacks[i][0]->pos) POND_LOG_RETURN("%s does not provide position feedback", motor_names[i].c_str());
        for (uint32_t i = 0; i < 12; i++) if (!feedbacks[i][0]->vel) POND_LOG_RETURN("%s does not provide velocity feedback", motor_names[i].c_str());
        for (uint32_t i = 0; i < 12; i++) *commands[i] = &motor_commands[i];

        std::vector<bool> modes; std::vector<Eigen::Vector3d> forces; std::vector<TrajectoryState> ref_state;
        {
            std::lock_guard<std::mutex> lock(mutex);

            if (ref_trajectory_states_buf.size() != 4 || modes_buf.size() != 4 || forces.size() !=4) return;

            modes = modes_buf;
            forces = forces_buf;
            ref_state = ref_trajectory_states_buf;
        }

        for (uint32_t i = 0; i < 4; i++)
        {
            Eigen::Vector3d joint_pos = {*feedbacks[3*i+0][0]->pos, *feedbacks[3*i+1][0]->pos, *feedbacks[3*i+2][0]->pos};
            Eigen::Vector3d joint_vel = {*feedbacks[3*i+0][0]->vel, *feedbacks[3*i+1][0]->vel, *feedbacks[3*i+2][0]->vel};
            Eigen::Vector3d torque;
            
            leg_config config;
            leg_dynamics dynamics;
            calculate_leg_dynamics(dynamics, config, joint_pos, joint_vel, Eigen::Vector3d(), modes[i], i%2 ? -1 : 1);

            if (modes[i])
            {
                Eigen::Matrix3d Kp, Kd;

                for (uint32_t i = 0; i < 3; i++)
                {
                    Kp(i, i) = natural_frequency * natural_frequency * dynamics.lambda(i, i);
                    Kd(i, i) = 2.0 * natural_frequency * dynamics.lambda(i, i);
                }

                torque = dynamics.jacobian.transpose() * (
                        Kp * (ref_state[i].pos - dynamics.p_foot) +
                        Kd * (ref_state[i].vel - dynamics.jacobian * joint_vel) +
                        dynamics.lambda * (ref_state[i].acc - dynamics.jacobian_dot * joint_vel)
                ) + dynamics.c_qdot + dynamics.g;
            }
            else torque = dynamics.jacobian.transpose() * forces[i];
            
            motor_commands[3*i+0].torque = torque[0];
            motor_commands[3*i+1].torque = torque[1];
            motor_commands[3*i+2].torque = torque[2];
        }
    });

    force_callback = createCallback<std::vector<Eigen::Vector3d>>({"ground_forces"}, [this](std::vector<Eigen::Vector3d>* forces_in) {
        std::lock_guard<std::mutex> lock(mutex);
        forces_buf = *forces_in;
    });

    mode_callback = createCallback<std::vector<bool>>({"phase_modes"}, [this](std::vector<bool>* modes_in) {
        std::lock_guard<std::mutex> lock(mutex);
        modes_buf = *modes_in;
    });

    trajectory_states_callback = createCallback<std::vector<TrajectoryState>>({"target_trajectory_states"}, [this](std::vector<TrajectoryState>* trajectory_states_in) {
        std::lock_guard<std::mutex> lock(mutex);
        ref_trajectory_states_buf = *trajectory_states_in;
    });

    return POND_SUCCESS;
}

void LegTorqueController::onShutdownTF()
{
    stage_callback.destroy();
    force_callback.destroy();
    mode_callback.destroy();
    trajectory_states_callback.destroy();
}
