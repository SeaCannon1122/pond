#include <pond/pond.hpp>
#include <pond/data_types/video_types.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

class GstServer : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
    GstElement* pipeline;
    GstElement* appsrc;
    pond::Receiver<ImgFrameSPtr> receiver;
    int32_t width, height, port;
    std::string ip;
    ImgFrame::Format image_format;
};

POND_MODULE_CPP_DECLARE(GstServer, "gst_server", "gst video streamer")

pond_result GstServer::onStartup(const std::vector<void*>& args)
{
    auto _width = parameter("width").asInt().getStrict();
    auto _height = parameter("height").asInt().getStrict();
    auto _port = parameter("port").asInt().getStrict();
    auto _ip = parameter("ip").asString().getStrict();

    auto _format_string = parameter("format").asString().getStrict({"RGB8", "BGR8", "Depth8", "Depth16", "Mono8", "Mono16"});
    if (!_format_string || !_width || !_height || !_port || !_ip) return POND_ERROR;
    width = *_width; height = *_height; port = *_port; ip = *_ip; image_format = ImgFrame::stringToFormat(*_format_string);

    receiver = createReceiver<ImgFrameSPtr>(
        {"in"},
        [this](ImgFrameSPtr& frame)
        {
            if (frame->width != width || frame->height != height || frame->format != image_format)
            {
                POND_LOG(
                    "ERROR: frame->width (%d) != width (%d) || frame->height (%d) != height (%d) ||  frame->format (%s) != image_format (%s)",
                    frame->width, width, frame->height, height, ImgFrame::formatToString(frame->format).c_str(), ImgFrame::formatToString(image_format).c_str()
                );
                return;
            }

            GstBuffer *buffer = gst_buffer_new_allocate(nullptr, frame->pixel_size * width * height, nullptr);
            GstMapInfo map;
            gst_buffer_map(buffer, &map, GST_MAP_WRITE);
            memcpy(map.data, frame->data, frame->pixel_size * width * height);

            gst_buffer_unmap(buffer, &map);
            gst_app_src_push_buffer(GST_APP_SRC(appsrc), buffer);
        }
    );

    GError *error = nullptr;
    if (!gst_is_initialized()) gst_init_check(0, NULL, &error);
    
    if (error)
    {
        POND_LOG("GError: %s", error->message);
        g_error_free(error);
        return POND_ERROR;
    }

    std::string gst_format;
    if (image_format == ImgFrame::Format::RGB8) gst_format = "RGB";
    if (image_format == ImgFrame::Format::BGR8) gst_format = "BGR";
    if (image_format == ImgFrame::Format::Depth8) gst_format = "GRAY8";
    if (image_format == ImgFrame::Format::Depth16) gst_format = "GRAY16_LE";
    if (image_format == ImgFrame::Format::Mono8) gst_format = "GRAY8";
    if (image_format == ImgFrame::Format::Mono16) gst_format = "GRAY16_LE";

    std::string pipeline_desc =
        "appsrc name=appsrc is-live=true block=false format=time do-timestamp=true "
        "caps=video/x-raw,format=" + gst_format + ",width=" + std::to_string(width) +
        ",height=" + std::to_string(height) + " "
        "! queue leaky=downstream max-size-buffers=1 max-size-time=0 max-size-bytes=0 "
        "! videoconvert "
        "! x264enc tune=zerolatency speed-preset=ultrafast key-int-max=" +
          std::to_string(parameter("key_int_max").asInt().get(30)) +
        " bitrate=" + std::to_string(parameter("bitrate").asInt().get(4000)) + " "
        "! rtph264pay pt=96 config-interval=1 "
        "! udpsink host=" + ip +
        " port=" + std::to_string(port) + " sync=false async=false";

    
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
