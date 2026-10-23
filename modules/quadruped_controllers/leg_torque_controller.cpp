#include "pond/h/module.h"
#include "pond/hpp/module_base.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <array>
#include <mutex>
#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>
#include <pond_data_types/motor_types.hpp>
#include <sophus/so3.hpp>
#include <vector>

class LegTorqueController : public pond::ModuleBaseTF
{
public:
    virtual pond_result onStartupTF(const std::vector<void*>& args) override;
    virtual void onShutdownTF() override;
private:
    pond::Callback force_callback, mode_callback, stage_callback;
    std::vector<Eigen::Vector3d> forces_buf; std::vector<bool> modes_buf;
    std::mutex mutex;

    double limb_length_hip_to_shoulder;
    double limb_length_thigh;
    double limb_length_shin;

    std::vector<std::string> motor_names;
    std::vector<MotorCommand> motor_commands;
    std::vector<JointState> joint_states;
};

POND_MODULE_CPP_DECLARE(LegTorqueController, "leg_torque_controller", "torque control block")

pond_result LegTorqueController::onStartupTF(const std::vector<void*>& args)
{
    forces_buf.resize(4, Eigen::Vector3d::Zero());
    modes_buf.resize(4, false);

    auto joint_names = parameter("joint_names").asStringArray().getStrict(12, 12); if (!joint_names) return POND_ERROR;
    auto motor_names_o = parameter("motor_names").asStringArray().getStrict(12, 12); if (!motor_names_o) return POND_ERROR;
    auto foot_link_names = parameter("foot_link_names").asStringArray().getStrict(4, 4); if (!foot_link_names) return POND_ERROR;
    
    motor_names = *motor_names_o;

    GetJointInfoRequest joint_requests[3];
    GetFrameTransformRequest tf_requests[4];

    for (uint32_t i = 0; i < 3; i++) if (!tfGetJointInfo(joint_names->at(i), joint_requests[i])) return POND_ERROR;
    
    if (!tfGetTransform(joint_requests[1].parent_link_name, joint_requests[0].child_link_name, 0, tf_requests[0])) return POND_ERROR;
    if (!tfGetTransform(joint_requests[2].parent_link_name, joint_requests[1].child_link_name, 0, tf_requests[1])) return POND_ERROR;
    if (!tfGetTransform(foot_link_names->at(0), joint_requests[2].child_link_name, 0, tf_requests[2])) return POND_ERROR;

    limb_length_hip_to_shoulder =   (tf_requests[0].tf * joint_requests[1].tf).translation().norm();
    limb_length_thigh =             (tf_requests[1].tf * joint_requests[2].tf).translation().norm();
    limb_length_shin =              (tf_requests[2].tf).translation().norm();

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
        for (uint32_t i = 0; i < 12; i++) *commands[i] = &motor_commands[i];

        std::array<bool, 4> modes; std::array<Eigen::Vector3d, 4> forces;
        {
            std::lock_guard<std::mutex> lock(mutex);
            std::copy(modes_buf.begin(), modes_buf.end(), modes.begin());
            std::copy(forces_buf.begin(), forces_buf.end(), forces.begin());
        }

        for (uint32_t i = 0; i < 4; i++)
        {
            Eigen::Vector3d p[4] = {Eigen::Vector3d::Zero()};
            Sophus::SO3d R[3];
            Eigen::Vector3d q = {*feedbacks[3*i+0][0]->pos, *feedbacks[3*i+1][0]->pos, *feedbacks[3*i+2][0]->pos};

            R[0] =               Sophus::SO3d::rotX(q[0]);
            p[1] = p[0] + R[0] * Eigen::Vector3d(0, limb_length_hip_to_shoulder, 0);;

            R[1] =        R[0] * Sophus::SO3d::rotY(q[1]);
            p[2] = p[1] + R[1] * Eigen::Vector3d(0, 0, -limb_length_thigh);

            R[2] =        R[1] * Sophus::SO3d::rotY(q[2]);
            p[3] = p[2] + R[2] * Eigen::Vector3d(0, 0, -limb_length_shin);

            Eigen::Vector3d w[3] = {
                       Eigen::Vector3d::UnitX(),
                R[0] * Eigen::Vector3d::UnitY(),
                R[1] * Eigen::Vector3d::UnitY()
            };

            Eigen::Matrix3d jacobian;
            jacobian.col(0) = w[0].cross(p[3] - p[0]);
            jacobian.col(1) = w[1].cross(p[3] - p[1]);
            jacobian.col(2) = w[2].cross(p[3] - p[2]);

            Eigen::Vector3d torque = jacobian.transpose() * forces[i];
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

    return POND_SUCCESS;
}

void LegTorqueController::onShutdownTF()
{
    stage_callback.destroy();
    force_callback.destroy();
    mode_callback.destroy();
}
