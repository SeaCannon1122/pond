#include <pond/pond.hpp>

class TemplateModule : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
};

POND_MODULE_CPP_DECLARE(TemplateModule, "template_module", "template info")

pond_result TemplateModule::onStartup(const std::vector<void*>& args)
{
    return POND_SUCCESS;
}

void TemplateModule::onShutdown()
{
}

void TemplateModule::onFrame()
{
}
