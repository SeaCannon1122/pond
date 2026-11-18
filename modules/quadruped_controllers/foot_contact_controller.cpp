#include <pond/pond.hpp>

class FootContactController : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
private:
};

POND_MODULE_CPP_DECLARE(FootContactController, "foot_contact_controller", "detects foot touchdown and lift")

pond_result FootContactController::onStartup(const std::vector<void*>& args)
{
    return POND_SUCCESS;
}

void FootContactController::onShutdown()
{
}