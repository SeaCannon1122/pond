#include "pond/hpp/dds.hpp"
#include <pond/pond.hpp>

class DDSTest : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;

    void callback_function(void** data)
    {
        
    }
};

POND_MODULE_CPP_DECLARE(DDSTest, "dds_test", "template module info")

POND_BUNDLE_DECLARE(
    "template bundle info",
    POND_MODULE(DDSTest),
)

pond_result DDSTest::onStartup(const std::vector<void*>& args)
{

    pond::ChannelsInfo().channels<int>("hello", "hello");
    std::array<std::string, 10> vals;
    pond::ChannelsInfo().channels<int>(vals);
    createControlCallback(
        
        [this]() {}
    );

    shutdown();

    return POND_SUCCESS;
}
