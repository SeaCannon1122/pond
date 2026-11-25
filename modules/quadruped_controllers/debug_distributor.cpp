#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>
#include <pond_data_types/motor_types.hpp>

#include "quadruped_robot_config.hpp"

class DebugDistributor : public pond::ModuleBaseTF
{
public:
    virtual pond_result onStartupTF(const std::vector<void*>& args) override;
    virtual void onShutdownTF() override;
private:
    std::vector<std::string> motor_names;
    std::string stage_name;

    QuadrupedRobotConfig cfg;

    std::vector<MotorCommand> commands;
    std::vector<MotorFeedback> feedbacks;
    std::vector<FrameTransform> poses;

    pond::Callback stage_callback;
    pond::DistributorTyped<std::vector<FrameTransform>> poses_distributor;
    pond::DistributorTyped<std::vector<MotorCommand>> cmd_distributor;
    pond::DistributorTyped<std::vector<MotorFeedback>> fb_distributor;
};

POND_MODULE_CPP_DECLARE(DebugDistributor, "debug_distributor", "updates the joint states of the robot tracker")

pond_result DebugDistributor::onStartupTF(const std::vector<void*>& args)
{
    stage_name = parameter("stage_name").asString().get("debug_distribute");

    auto motor_names_o = parameter("motor_names").asStringArray().getStrict(12, 12); if (!motor_names_o) return POND_ERROR;
    motor_names = *motor_names_o;

    poses.resize(4);
    commands.resize(12);
    feedbacks.resize(12);

    if (!cfg.init(*this)) return POND_ERROR;

    poses_distributor = createDistributorTyped<std::vector<FrameTransform>>({"debug/foot_poses"});
    cmd_distributor = createDistributorTyped<std::vector<MotorCommand>>({"debug/motor_commands"});
    fb_distributor = createDistributorTyped<std::vector<MotorFeedback>>({"debug/motor_feedbacks"});

    pond::ChannelsInfo cb_info;

    for (uint32_t i = 0; i < 12; i++) cb_info.channel<MotorFeedback*>(stage_name + "/" + motor_names[i] + "_feedback");
    for (uint32_t i = 0; i < 12; i++) cb_info.channel<MotorCommand*>(stage_name + "/" + motor_names[i] + "_command");

    
    stage_callback = createCallback<void>(cb_info, [this](void** data)
        {
            MotorFeedback*** feedbacks_in = (MotorFeedback***)data;
            MotorCommand*** commands_in = (MotorCommand***)&data[12];

            std::array<Eigen::Vector3d, 4> positions;

            for (uint32_t i = 0; i < 12; i++)
            {
                if (*feedbacks_in[i] == NULL) POND_LOG_RETURN("%s is NULL", (stage_name + "/" + motor_names[i] + "_feedback").c_str());
                if (!feedbacks_in[i][0]->pos) POND_LOG_RETURN("%s does not provide position feedback", motor_names[i].c_str());
                if (*commands_in[i] == NULL) POND_LOG_RETURN("%s is NULL", (stage_name + "/" + motor_names[i] + "_command").c_str());

                feedbacks[i] = *feedbacks_in[i][0];
                positions[i/3][i%3] = *feedbacks_in[i][0]->pos;
                commands[i] = *commands_in[i][0];
            }

            for (uint32_t i = 0; i < 4; i++)
            {
                Sophus::SO3d R[3];
                Eigen::Vector3d p[4] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

                R[0] = Sophus::SO3d::rotX(positions[i][0]);
                p[1] = p[0] + R[0] * cfg.legs[i].segments[0].offset;

                R[1] = R[0] * Sophus::SO3d::rotY(positions[i][1]);
                p[2] = p[1] + R[1] * cfg.legs[i].segments[1].offset;

                R[2] = R[1] * Sophus::SO3d::rotY(positions[i][2]);
                p[3] = p[2] + R[2] * cfg.legs[i].segments[2].offset;

                poses[i].tf.translation() =
                    Sophus::SO3d::rotX(positions[i][0]) * (
                        cfg.legs[i].segments[0].offset + Sophus::SO3d::rotY(positions[i][1]) * (
                            cfg.legs[i].segments[1].offset + Sophus::SO3d::rotY(positions[i][2]) * (
                                cfg.legs[i].segments[2].offset
                            )
                        )
                    ) +
                    cfg.legs[i].origin_pos;

                poses[i].stamp.hw_time = poses[i].stamp.time = feedbacks_in[i][0]->time;
            }
            
            poses_distributor.distribute(&poses);
            fb_distributor.distribute(&feedbacks);
            cmd_distributor.distribute(&commands);
        }
    );

    return POND_SUCCESS;
}

void DebugDistributor::onShutdownTF()
{
    stage_callback.destroy();

    poses_distributor.destroy();
    cmd_distributor.destroy();
    fb_distributor.destroy();
}