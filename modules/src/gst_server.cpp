#include "pond/pond.h"
#include <cstdint>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <pond/pond.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <pond/data_types.hpp>

class GstServer : public pond::ModuleBase
{
public:
    virtual pond_result onStartup() override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
    GstElement* pipeline;
    GstElement* appsrc;
    pond::Receiver<ImgFrameSPtr> receiver;
    int32_t width, height, fps, port;
    std::string ip;
};

POND_MODULE_CPP_DECLARE(GstServer, "gst_server", "gst video streamer")

pond_result GstServer::onStartup()
{
    width = parameter("width").asInt().get(640);
    height = parameter("height").asInt().get(480);
    fps = parameter("fps").asInt().get(30);
    port = parameter("port").asInt().get(5000);
    ip = parameter("ip").asString().get("127.0.0.1");

    receiver = createReceiver<ImgFrameSPtr>(
        {"in"},
        [this](ImgFrameSPtr& frame)
        {
            if (frame->width != width || frame->height != height || frame->pixel_size != 3)
            {
                POND_LOG(
                    "ERROR: frame->width (%d) != width (%d) || frame->height (%d) != height (%d) || frame->pixel_size (%d) != 3",
                    frame->width, width, frame->height, height, frame->pixel_size
                );
                return;
            }

            GstBuffer *buffer = gst_buffer_new_allocate(nullptr, 3 * width * height, nullptr);
            GstMapInfo map;
            gst_buffer_map(buffer, &map, GST_MAP_WRITE);
            memcpy(map.data, frame->data, 3 * width * height);

            gst_buffer_unmap(buffer, &map);
            gst_app_src_push_buffer(GST_APP_SRC(appsrc), buffer);
        }
    );

    gst_init(0, NULL);

    std::string pipeline_desc =
        "appsrc name=appsrc is-live=true block=false format=time do-timestamp=true "
        "caps=video/x-raw,format=BGR,width=" + std::to_string(width) +
        ",height=" + std::to_string(height) +
        ",framerate=" + std::to_string(fps) + "/1 "
        "! queue leaky=downstream max-size-buffers=1 max-size-time=0 max-size-bytes=0 "
        "! videoconvert "
        "! x264enc tune=zerolatency speed-preset=ultrafast key-int-max=" +
          std::to_string(parameter("key_int_max").asInt().get(30)) +
        " bitrate=" + std::to_string(parameter("bitrate").asInt().get(4000)) + " "
        "! rtph264pay pt=96 config-interval=1 "
        "! udpsink host=" + ip +
        " port=" + std::to_string(port) + " sync=false async=false";

    GError *error = nullptr;
    pipeline = gst_parse_launch(pipeline_desc.c_str(), &error);

    if (!pipeline || error)
    {
        POND_LOG("Failed to create Gstreamer pipeline %s", pipeline_desc.c_str());
        if (error)
        {
            POND_LOG("GError: %s", error->message);
            g_error_free(error);
        }
        return POND_ERROR;
    }

    appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "appsrc");
    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    return POND_SUCCESS;
}

void GstServer::onShutdown()
{
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    receiver.destroy();
}

void GstServer::onFrame()
{
    
}
