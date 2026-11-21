#include <Eigen/src/Core/Matrix.h>
#include <optional>
#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>
#include <pond_data_types/motor_types.hpp>

class MotorAbstractor : public pond::ModuleBaseTF
{
public:
    virtual pond_result onStartupTF(const std::vector<void*>& args) override;
    virtual void onShutdownTF() override;
private:
    std::vector<JointState> joint_states;
    std::vector<FrameTransform> distributed_poses;
    std::vector<std::string> motor_names;
    std::string fb_stage_name, cmd_stage_name;
    bool torque_mode, disable = false; std::atomic<bool> disable_rec;

    struct
    {
        Eigen::Vector3d origin_pos;
        Eigen::Vector3d segement_offsets[3];
    } configs[4];

    std::array<Eigen::Vector3d, 4> positions, velocities, cmd_positions_or_torques;
    MotorCommand commands[12];

    pond::Callback fb_stage_callback, cmd_stage_callback, disable_callback;
    pond::DistributorTyped<std::vector<FrameTransform>> poses_distributor;
};

POND_MODULE_CPP_DECLARE(MotorAbstractor, "motor_abstractor", "updates the joint states of the robot tracker")

pond_result MotorAbstractor::onStartupTF(const std::vector<void*>& args)
{
    fb_stage_name = parameter("feedback_stage_name").asString().get("convert_from_motor_feedback");
    cmd_stage_name = parameter("command_stage_name").asString().get("convert_to_motor_commands");
    torque_mode = parameter("torque_mode").asBool().get(true);

    auto reference_link = parameter("reference_link").asString().get("base_link");
    auto joint_names = parameter("joint_names").asStringArray().getStrict(); if (!joint_names) return POND_ERROR;
    auto motor_names_o = parameter("motor_names").asStringArray().getStrict(joint_names->size(), joint_names->size()); if (!motor_names_o) return POND_ERROR;
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

        configs[l].segement_offsets[0] = (tf_requests[0].tf * joint_requests[1].tf).translation();
        configs[l].segement_offsets[1] = (tf_requests[1].tf * joint_requests[2].tf).translation();
        configs[l].segement_offsets[2] = (tf_requests[2].tf).translation();

        if (!tfGetTransform(joint_requests[0].parent_link_name, reference_link, 0, tf_requests[3])) return POND_ERROR;
        configs[l].origin_pos = tf_requests[3].tf * joint_requests[0].tf * Eigen::Vector3d::Zero();
    }

    distributed_poses.resize(4, {.stamp = {.frame_id = reference_link}});
    joint_states.resize(joint_names->size());

    for (uint32_t i = 0; i < 12; i++)
    {
        joint_states[i].joint_name = joint_names->at(i);
        motor_names[i] = motor_names_o->at(i);
    }

    poses_distributor = createDistributorTyped<std::vector<FrameTransform>>({"foot_poses"});

    pond::ChannelsInfo fb_info, cmd_info;
    fb_info.channels<std::array<Eigen::Vector3d, 4>>(fb_stage_name+"/joint_positions", fb_stage_name+"/joint_velocities");
    cmd_info.channel<std::array<Eigen::Vector3d, 4>>(cmd_stage_name+(torque_mode ? "/joint_target_torques" : "/joint_target_positions"));

    for (uint32_t i = 0; i < 12; i++)
    {
        fb_info.channel<MotorFeedback*>(fb_stage_name + "/" + motor_names[i] + "_feedback");
        cmd_info.channel<MotorCommand*>(cmd_stage_name + "/" + motor_names[i] + "_command");
    }
    
    fb_stage_callback = createCallback<void>(fb_info, [this](void** data)
        {
            MotorFeedback*** feedbacks = (MotorFeedback***)&data[2];
            for (uint32_t i = 0; i < 12; i++)
            {
                if (*feedbacks[i] == NULL) POND_LOG_RETURN("%s is NULL", (fb_stage_name + "/" + motor_names[i] + "_feedback").c_str());
                if (!feedbacks[i][0]->pos) POND_LOG_RETURN("%s does not provide position feedback", motor_names[i].c_str());
                if (!feedbacks[i][0]->vel) POND_LOG_RETURN("%s does not provide velocity feedback", motor_names[i].c_str());
                
                joint_states[i].time = feedbacks[i][0]->time;
                joint_states[i].hw_time = feedbacks[i][0]->hw_time;
                joint_states[i].angle = positions[i/3][i%3] = *feedbacks[i][0]->pos;
                                       velocities[i/3][i%3] = *feedbacks[i][0]->vel;
            }

            *((std::array<Eigen::Vector3d, 4>***)data)[0] = &positions;
            *((std::array<Eigen::Vector3d, 4>***)data)[1] = &velocities;

            tfSetJointStates(joint_states);

            for (uint32_t i = 0; i < 4; i++)
            {
                Sophus::SO3d R[3];
                Eigen::Vector3d p[4] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

                // Apply side_sign to the lateral hip offset vector
                R[0] = Sophus::SO3d::rotX(positions[i][0]);
                p[1] = p[0] + R[0] * configs[i].segement_offsets[0];

                R[1] = R[0] * Sophus::SO3d::rotY(positions[i][1]);
                p[2] = p[1] + R[1] * configs[i].segement_offsets[1];

                R[2] = R[1] * Sophus::SO3d::rotY(positions[i][2]);
                p[3] = p[2] + R[2] * configs[i].segement_offsets[2];

                distributed_poses[i].tf.translation() =
                    Sophus::SO3d::rotX(positions[i][0]) * (
                        configs[i].segement_offsets[0] + Sophus::SO3d::rotY(positions[i][1]) * (
                            configs[i].segement_offsets[1] + Sophus::SO3d::rotY(positions[i][2]) * (
                                configs[i].segement_offsets[2]
                            )
                        )
                    ) +
                    configs[i].origin_pos;

                distributed_poses[i].stamp.hw_time = distributed_poses[i].stamp.time = joint_states[i*3].time;
            }
            poses_distributor.distribute(&distributed_poses);
        }
    );

    cmd_stage_callback = createCallback<void>(cmd_info, [this](void** data)
        {
            std::array<Eigen::Vector3d, 4>** positions_or_torques = (std::array<Eigen::Vector3d, 4>**)*data;
            if (*positions_or_torques == NULL) POND_LOG_RETURN("%s/%s is NULL", cmd_stage_name.c_str(), (torque_mode ? "/joint_target_torques" : "/joint_target_positions"));

            bool disable_flag = disable_rec.load();

            for (uint32_t i = 0; i < 12; i++)
            {
                *((MotorCommand***)data)[i+1] = &commands[i];

                if (torque_mode) commands[i].torque = positions_or_torques[0][0][i/3][i%3];
                else             commands[i].pos    = positions_or_torques[0][0][i/3][i%3];

                if (disable_flag != disable) commands[i].disable = disable_flag;
                else                         commands[i].disable = std::nullopt;
            }

            disable = disable_flag;
        }
    );

    disable_callback = createCallback<bool>({"disable_legs"}, [this](bool* flag) {disable_rec.store(*flag);});

    return POND_SUCCESS;
}

void MotorAbstractor::onShutdownTF()
{
    disable_callback.destroy();
    fb_stage_callback.destroy();
    cmd_stage_callback.destroy();
    poses_distributor.destroy();
}