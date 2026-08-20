#include <pond/pond.hpp>

class Ros2Bridge : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
};

POND_MODULE_CPP_DECLARE(Ros2Bridge, "ros2_bridge", "bridging different message types to ros2")

pond_result Ros2Bridge::onStartup(const std::vector<void*>& args)
{
    return POND_SUCCESS;
}

void Ros2Bridge::onShutdown()
{
}

void Ros2Bridge::onFrame()
{
}
