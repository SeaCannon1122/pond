#pragma once

#include <pond/pond.hpp>

class TemplateModule : public pond::ModuleBase
{
public:
    virtual pond_result onStartup() override;
    virtual void onShutdown() override;
    virtual pond_result onActivate() override;
    virtual void onDeactivate() override;
    virtual void onFrame() override;
private:

};