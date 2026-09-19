#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>
#include <pond_data_types/motor_types.hpp>

class AngleGripperController : public pond::ModuleBaseTF
{
public:
    virtual pond_result onStartupTF(const std::vector<void*>& args) override;
    virtual void onShutdownTF() override;
private:

    pond::Receiver width_receiver, disable_receiver, update_receiver;
    std::string gripper_joint;

    std::atomic<double> received_width{-1}, received_disable{false};
    double radius, offset;
    bool disabled = false;
};

POND_MODULE_CPP_DECLARE(AngleGripperController, "angle_gripper_controller", "Control an angle gripper")

inline double safe_asin(double x) { return std::asin(std::clamp(x, -1.0, 1.0)); }

pond_result AngleGripperController::onStartupTF(const std::vector<void*>& args)
{
    gripper_joint = parameter("joint_name").asString().get("gripper_joint");

    radius = parameter("radius").asDouble().get(0.1);
    offset = parameter("offset").asDouble().get(0.1);
    
    update_receiver = createReceiver<MotorInterface>({"gripper_motor/update"}, [this](MotorInterface* interface) {

        tfSetJointState(gripper_joint, interface->feedback.pos ? *interface->feedback.pos : 0, interface->feedback.time);

        if (received_disable.load())
        {
            if (!disabled) interface->command.disable = true;
            disabled = true;
            received_width.store(-1);
        }
        else
        {
            if (disabled) interface->command.disable = false;
            disabled = false;

            double width = received_width.load();
            if (width != -1) interface->command.pos = safe_asin((width + 2.0 * offset) / radius / 2);
        }
    });

    width_receiver = createReceiver<double>({"gripper_width"}, [this](double* width) {received_width.store(*width);});
    disable_receiver = createReceiver<bool>({"disable_gripper"}, [this](bool* disable) {received_disable.store(*disable);});

    return POND_SUCCESS;
}

void AngleGripperController::onShutdownTF()
{
    update_receiver.destroy();
    width_receiver.destroy();
    disable_receiver.destroy();
}