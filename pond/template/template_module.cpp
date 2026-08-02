#include "template_module.hpp"

pond_result TemplateModule::onShutdown()
{
    return POND_SUCCESS;
}

void TemplateModule::onShutdown()
{
    
}

pond_result TemplateModule::onActivate()
{
    return POND_SUCCESS;
}

void TemplateModule::onDeactivate()
{

}

void TemplateModule::onFrame()
{
    
}

POND_MODULE_CPP_DECLARE(TemplateModule)