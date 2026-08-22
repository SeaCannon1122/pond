#define POND_MODULE_CPP_MAKE_IMPLEMENTATION
#include <pond/pond.hpp>

class RTABMap : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
};

POND_MODULE_CPP_DECLARE(RTABMap, "rtabmap", "rtabmap module")

POND_BUNDLE_DECLARE(
    "rtabmap", 
    1,
    POND_MODULE(RTABMap),
)

pond_result RTABMap::onStartup(const std::vector<void*>& args)
{
    return POND_SUCCESS;
}

void RTABMap::onShutdown()
{
}

void RTABMap::onFrame()
{
}
