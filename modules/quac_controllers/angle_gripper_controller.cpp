#include <optional>
#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>
#include <pond_data_types/motor_types.hpp>

class AngleGripperController : public pond::ModuleBaseTF
{
public:
    virtual pond_result onStartupTF(const std::vector<void*>& args) override;
    virtual void onShutdownTF() override;
private:

    pond::Callback width_callback, update_callback;
    std::string gripper_joint;

    MotorCommand motor_command;

    std::atomic<double> received_width{-1};
    double radius, offset;
};

POND_MODULE_CPP_DECLARE(AngleGripperController, "angle_gripper_controller", "Control an angle gripper")

inline double safe_asin(double x) { return std::asin(std::clamp(x, -1.0, 1.0)); }

pond_result AngleGripperController::onStartupTF(const std::vector<void*>& args)
{
    gripper_joint = parameter("joint_name").asString().get("gripper_joint");

    radius = parameter("radius").asDouble().get(0.1);
    offset = parameter("offset").asDouble().get(0.1);
    
    update_callback = createCallback<MotorCommand*, MotorFeedback*>({"control_gripper/gripper_motor_command", "control_gripper/gripper_motor_feedback"}, [this](MotorCommand** cmd, MotorFeedback** fb) {

        if (*fb == NULL) POND_LOG_RETURN("%s is NULL", "control_gripper/gripper_motor_feedback");
        *cmd = &motor_command;

        tfSetJointState(gripper_joint, fb[0]->pos ? *fb[0]->pos : 0, fb[0]->time);

        double width = received_width.load();
        if (width != -1) motor_command.pos = safe_asin((width + 2.0 * offset) / radius / 2);
        else motor_command.pos = std::nullopt;
    });

    width_callback = createCallback<double>({"gripper_width"}, [this](double* width) {received_width.store(*width);});

    return POND_SUCCESS;
}

void AngleGripperController::onShutdownTF()
{
    update_callback.destroy();
    width_callback.destroy();
}