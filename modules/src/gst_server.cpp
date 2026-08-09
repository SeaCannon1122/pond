#include <pond/pond.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <quac_modules/interfaces/wrapped_image_frame.hpp>

class GstServer : public pond::ModuleBase
{
public:
    virtual pond_result onStartup() override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
    GstElement* pipeline;
    GstElement* appsrc;
    pond::Receiver receiver;
};

POND_MODULE_CPP_DECLARE(GstServer, "gst_server", "gst video streamer")

pond_result GstServer::onStartup()
{
    POND_LOG("activating ...");
    receiver = createReceiver("in", [this](void* data)
        {
            WrappedImageFrame* frame = (WrappedImageFrame*)data;

            POND_LOG("width %d height %d", frame->width, frame->height);
        }
    );

    gst_init(0, NULL);

    POND_LOG("activated");
    return POND_SUCCESS;
}

void GstServer::onShutdown()
{
    POND_LOG("deactivating ...");
    receiver.destroy();
    POND_LOG("deactivated");
}

void GstServer::onFrame()
{
    
}
