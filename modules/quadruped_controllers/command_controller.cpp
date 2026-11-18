#include <pond/pond.hpp>

class CommandController : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
private:
};

POND_MODULE_CPP_DECLARE(CommandController, "command_controller", "gathers robot commands")

pond_result CommandController::onStartup(const std::vector<void*>& args)
{
    return POND_SUCCESS;
}

void CommandController::onShutdown()
{
}