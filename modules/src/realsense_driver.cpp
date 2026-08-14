#include <pond/pond.hpp>
#include <librealsense2/rs.hpp>
#include <pond/data_types.hpp>
#include <memory>

class RealSenseImgFrame : public ImgFrame
{
public:

    explicit RealSenseImgFrame(rs2::video_frame& frame_, const std::string& format_) : frame(frame_)
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

class RealsenseDriver : public pond::ModuleBase
{
public:
    virtual pond_result onStartup() override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
    rs2::pipeline pipe;
    rs2::config cfg;
    pond::Distributor<ImgFrameSPtr, ImgFrameSPtr, ImgFrameSPtr, ImgFrameSPtr> distributor;
};

POND_MODULE_CPP_DECLARE(RealsenseDriver, "realsense_driver", "driver for the intel realsense d435")

pond_result RealsenseDriver::onStartup()
{
    POND_LOG("activating ...");

    distributor = createDistributor<ImgFrameSPtr, ImgFrameSPtr, ImgFrameSPtr, ImgFrameSPtr>({"color", "depth", "mono_left", "mono_right"});
    
    std::vector<int32_t> color_dims = *parameter("color.dims").getIntArray({1280, 720}, 2, 2);
    std::vector<int32_t> depth_dims = *parameter("depth.dims").getIntArray({640, 480}, 2, 2);
    std::vector<int32_t> mono_dims = *parameter("mono.dims").getIntArray({640, 480}, 2, 2);
    uint32_t fps = *parameter("fps").getInt(30);

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

    POND_LOG("activated");
    return POND_SUCCESS;
}

void RealsenseDriver::onShutdown()
{
    POND_LOG("shutting down ...");
    pipe.stop();
    distributor.destroy();
    POND_LOG("shut down");
}

void RealsenseDriver::onFrame()
{
    rs2::frameset frames = pipe.wait_for_frames();
    rs2::video_frame color_frame = frames.get_color_frame();
    rs2::depth_frame depth_frame = frames.get_depth_frame();
    rs2::video_frame left_frame = frames.get_infrared_frame(1);
    rs2::video_frame right_frame = frames.get_infrared_frame(2);

    ImgFrameSPtr color_msg = std::make_shared<RealSenseImgFrame>(color_frame, "rgb8");
    ImgFrameSPtr depth_msg = std::make_shared<RealSenseImgFrame>(depth_frame, "mono16");
    ImgFrameSPtr left_msg = std::make_shared<RealSenseImgFrame>(left_frame, "mono8");
    ImgFrameSPtr right_msg = std::make_shared<RealSenseImgFrame>(right_frame, "mono8");

    distributor.distribute(color_msg, depth_msg, left_msg, right_msg);
}