#include <pond/pond.hpp>
#include <pond/data_types/video_types.hpp>

#include <librealsense2/rs.hpp>

class RealSenseImgFrame : public ImgFrame
{
public:

    explicit RealSenseImgFrame(rs2::video_frame& frame_, ImgFrame::Format format_, double timestamp, double depth_scale_ = 1) : frame(frame_)
    {

        data = (void*)frame.get_data();
        width = frame.get_width();
        height = frame.get_height();
        pixel_size = frame.get_bytes_per_pixel();
        format = format_;
        depth_scale = depth_scale_;
        stamp.hw_time = frame.get_timestamp();
        stamp.time = timestamp;
    }

private:
    rs2::video_frame frame;
};

class RealsenseCamera : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
    rs2::pipeline pipe;
    rs2::config cfg;
    pond::Distributor<ImgFrameSPtr, CameraInfo, ImgFrameSPtr, CameraInfo, ImgFrameSPtr, CameraInfo, ImgFrameSPtr, CameraInfo> distributor;
    CameraInfo color_info, depth_info, mono_left_info, mono_right_info;
};

POND_MODULE_CPP_DECLARE(RealsenseCamera, "realsense_camera", "driver for the intel realsense d435")

pond_result RealsenseCamera::onStartup(const std::vector<void*>& args)
{
    
    std::vector<int32_t> color_dims = parameter("color.dims").asIntArray().get({1280, 720}, 2, 2);
    std::vector<int32_t> depth_dims = parameter("depth.dims").asIntArray().get({640, 480}, 2, 2);
    std::vector<int32_t> mono_dims = parameter("mono.dims").asIntArray().get({640, 480}, 2, 2);
    uint32_t fps = parameter("fps").asInt().get(30);

    if (auto serial_number = parameter("serial_number").asString().getStrict({}, false))
    {
        rs2::context ctx;
        rs2::device_list devices = ctx.query_devices();
        POND_LOG("%d device(s) connected", devices.size());

        bool connected = false;
        for (auto&& dev : devices) if (dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) == serial_number)
        {
            connected = true;
            cfg.enable_device(*serial_number);
            break;
        }

        if (!connected)
        {
            POND_LOG("No device with serial number '%s' connected", serial_number->c_str());
            return POND_ERROR;
        }
    }

    cfg.enable_stream(
        RS2_STREAM_INFRARED,
        1,
        mono_dims[0],
        mono_dims[1],
        RS2_FORMAT_Y8,
        fps
    );

    cfg.enable_stream(
        RS2_STREAM_INFRARED,
        2,
        mono_dims[0],
        mono_dims[1],
        RS2_FORMAT_Y8,
        fps
    );

    cfg.enable_stream(
        RS2_STREAM_COLOR, 
        color_dims[0], 
        color_dims[1], 
        RS2_FORMAT_RGB8,
        fps
    );

    cfg.enable_stream(
        RS2_STREAM_DEPTH,
        depth_dims[0],
        depth_dims[1],
        RS2_FORMAT_Z16,
        fps
    );
        
    rs2::pipeline_profile profile = pipe.start(cfg);

    for (auto&& sensor : profile.get_device().query_sensors())
        if (sensor.supports(RS2_OPTION_EMITTER_ENABLED))
            sensor.set_option(RS2_OPTION_EMITTER_ENABLED, parameter("disable_emitter").asBool().get(false) ? 0 : 1);

    distributor = createDistributor<ImgFrameSPtr, CameraInfo, ImgFrameSPtr, CameraInfo, ImgFrameSPtr, CameraInfo, ImgFrameSPtr, CameraInfo>(
        {
            "color/image", "color/cam_info", 
            "depth/image", "depth/cam_info", 
            "mono_left/image", "mono_left/cam_info", 
            "mono_right/image", "mono_right/cam_info"
        }
    );

    return POND_SUCCESS;
}

void RealsenseCamera::onShutdown()
{
    pipe.stop();
    distributor.destroy();
}

void RealsenseCamera::onFrame()
{
    rs2::frameset frames = pipe.wait_for_frames();
    double time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    
    rs2::video_frame color_frame = frames.get_color_frame();
    rs2::depth_frame depth_frame = frames.get_depth_frame();
    rs2::video_frame left_frame = frames.get_infrared_frame(1);
    rs2::video_frame right_frame = frames.get_infrared_frame(2);

    ImgFrameSPtr color_msg = std::make_shared<RealSenseImgFrame>(color_frame, ImgFrame::Format::RGB8, time);
    ImgFrameSPtr depth_msg = std::make_shared<RealSenseImgFrame>(depth_frame, ImgFrame::Format::Depth16, time, depth_frame.get_units());
    ImgFrameSPtr left_msg = std::make_shared<RealSenseImgFrame>(left_frame, ImgFrame::Format::Mono8, time);
    ImgFrameSPtr right_msg = std::make_shared<RealSenseImgFrame>(right_frame, ImgFrame::Format::Mono8, time);

    distributor.distribute(color_msg, color_info, depth_msg, depth_info, left_msg, mono_left_info, right_msg, mono_right_info);
}