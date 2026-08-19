#include <pond/pond.hpp>
#include <librealsense2/rs.hpp>
#include <pond/data_types.hpp>
#include <memory>

class RealSenseImgFrame : public ImgFrame
{
public:

    explicit RealSenseImgFrame(rs2::video_frame& frame_, ImgFrame::Format format_) : frame(frame_)
    {

        data = frame.get_data();
        width = frame.get_width();
        height = frame.get_height();
        pixel_size = frame.get_bytes_per_pixel();
        format = format_;
    }

private:
    rs2::video_frame frame;
};

class RealsenseCamera : public pond::ModuleBase
{
public:
    virtual pond_result onStartup() override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
    rs2::pipeline pipe;
    rs2::config cfg;
    pond::Distributor<ImgFrameSPtr, CameraInfo, ImgFrameSPtr, CameraInfo, ImgFrameSPtr, CameraInfo, ImgFrameSPtr, CameraInfo> distributor;
    CameraInfo color_info, depth_info, mono_left_info, mono_right_info;
};

POND_MODULE_CPP_DECLARE(RealsenseCamera, "realsense_camera", "driver for the intel realsense d435")

pond_result RealsenseCamera::onStartup()
{
    distributor = createDistributor<ImgFrameSPtr, CameraInfo, ImgFrameSPtr, CameraInfo, ImgFrameSPtr, CameraInfo, ImgFrameSPtr, CameraInfo>(
        {
            "color/image", "color/cam_info", 
            "depth/image", "depth/cam_info", 
            "mono_left/image", "mono_left/cam_info", 
            "mono_right/image", "mono_right/cam_info"
        }
    );
    
    std::vector<int32_t> color_dims = parameter("color.dims").asIntArray().get({1280, 720}, 2, 2);
    std::vector<int32_t> depth_dims = parameter("depth.dims").asIntArray().get({640, 480}, 2, 2);
    std::vector<int32_t> mono_dims = parameter("mono.dims").asIntArray().get({640, 480}, 2, 2);
    uint32_t fps = parameter("fps").asInt().get(30);

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
    {
        if (sensor.supports(RS2_OPTION_EMITTER_ENABLED)) sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0);
    }

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
    rs2::video_frame color_frame = frames.get_color_frame();
    rs2::depth_frame depth_frame = frames.get_depth_frame();
    rs2::video_frame left_frame = frames.get_infrared_frame(1);
    rs2::video_frame right_frame = frames.get_infrared_frame(2);

    ImgFrameSPtr color_msg = std::make_shared<RealSenseImgFrame>(color_frame, ImgFrame::Format::RGB8);
    ImgFrameSPtr depth_msg = std::make_shared<RealSenseImgFrame>(depth_frame, ImgFrame::Format::Depth16);
    ImgFrameSPtr left_msg = std::make_shared<RealSenseImgFrame>(left_frame, ImgFrame::Format::Mono8);
    ImgFrameSPtr right_msg = std::make_shared<RealSenseImgFrame>(right_frame, ImgFrame::Format::Mono8);

    distributor.distribute(color_msg, color_info, depth_msg, depth_info, left_msg, mono_left_info, right_msg, mono_right_info);
}