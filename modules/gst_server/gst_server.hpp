#include <pond/pond.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

class GstServer : public pond::ModuleBase
{
public:
    GstServer();
    virtual pond_result onStartup() override;
    virtual void onShutdown() override;
    virtual pond_result onActivate() override;
    virtual void onDeactivate() override;
    virtual void onFrame() override;
private:
    GstElement* pipeline;
    GstElement* appsrc;
};