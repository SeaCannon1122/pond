#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>
#include <pond_data_types/motor_types.hpp>

class MotorJointUpdater : public pond::ModuleBaseTF
{
public:
    virtual pond_result onStartupTF(const std::vector<void*>& args) override;
    virtual void onShutdownTF() override;
private:
    std::vector<JointState> joint_states;
    std::vector<std::string> motor_names;
    std::string stage_name;

    pond::Callback stage_callback;
};

POND_MODULE_CPP_DECLARE(MotorJointUpdater, "motor_joint_updater", "updates the joint states of the robot tracker")

pond_result MotorJointUpdater::onStartupTF(const std::vector<void*>& args)
{
    stage_name = parameter("stage_name").asString().get("update_joint_states");

    auto joint_names = parameter("joint_names").asStringArray().getStrict(); if (!joint_names) return POND_ERROR;
    auto motor_names_o = parameter("motor_names").asStringArray().getStrict(joint_names->size(), joint_names->size()); if (!motor_names_o) return POND_ERROR;
    motor_names = *motor_names_o;

    joint_states.resize(joint_names->size());

    pond::ChannelsInfo info;
    for (uint32_t i = 0; i < 12; i++)
    {
        joint_states[i].joint_name = joint_names->at(i);
        motor_names[i] = motor_names_o->at(i);
        info.channel<MotorFeedback*>(stage_name + "/" + motor_names[i] + "_feedback");
    }

    stage_callback = createCallback<MotorFeedback*>(info, [this](MotorFeedback*** feedbacks_in)
        {
            for (uint32_t i = 0; i < 12; i++) if (*feedbacks_in[i] == NULL) POND_LOG_RETURN("%s is NULL", (stage_name + "/" + motor_names[i] + "_feedback").c_str());

            for (uint32_t i = 0; i < 12; i++)
            {
                if (feedbacks_in[i][0]->pos) joint_states[i].angle = *feedbacks_in[i][0]->pos;
                joint_states[i].time = feedbacks_in[i][0]->time;
                joint_states[i].hw_time = feedbacks_in[i][0]->hw_time;
            }

            tfSetJointStates(joint_states);
        }
    );

    return POND_SUCCESS;
}

void MotorJointUpdater::onShutdownTF()
{
    stage_callback.destroy();
}