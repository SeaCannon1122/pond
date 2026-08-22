#include <pond/pond.hpp>
#include <pond/data_types/command_types.hpp>
#include <mutex>

class DiffDriveController : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
    std::mutex mutex;
    TwistCommand cmd;

    pond::Receiver<TwistCommand> receiver;
};

POND_MODULE_CPP_DECLARE(DiffDriveController, "diff_drive_controller", "Control a differential drive robot")

pond_result DiffDriveController::onStartup(const std::vector<void*>& args)
{
    
    receiver = createReceiver<TwistCommand>({"cmd_vel"}, [](TwistCommand* twist)
        {

        }
    );

    return POND_SUCCESS;
}

void DiffDriveController::onShutdown()
{

}

void DiffDriveController::onFrame()
{
}
