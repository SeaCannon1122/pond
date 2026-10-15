#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>

class GetTFTest : public pond::ModuleBaseTF
{
public:
    virtual pond_result onStartupTF(const std::vector<void*>& args) override;
    virtual void onShutdownTF() override;
    virtual void onFrame() override;
private:
};

POND_MODULE_CPP_DECLARE(GetTFTest, "get_tf_test", "template module info")

POND_BUNDLE_DECLARE(
    "template bundle info",
    POND_MODULE(GetTFTest),
)

pond_result GetTFTest::onStartupTF(const std::vector<void*>& args)
{

    GetFrameTransformRequest request;
    if (tfGetTransform("camera_back_color_optical_frame", "base_link", pond::get_time(), request)) POND_LOG("SUCCESS");
    else POND_LOG("FAIL");

    shutdown();

    return POND_SUCCESS;
}

void GetTFTest::onShutdownTF()
{
}

void GetTFTest::onFrame()
{
}
