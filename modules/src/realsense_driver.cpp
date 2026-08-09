#include <pond/pond.hpp>
#include <librealsense2/rs.hpp>
#include <quac_modules/interfaces//wrapped_image_frame.hpp>
#include <memory>

class RealSenseWrappedImageFrame : public WrappedImageFrame
{
public:

    explicit RealSenseWrappedImageFrame(rs2::video_frame& frame_, const std::string& format_) : frame(frame_)
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
    pond::Distributor color_distributor;
    pond::Distributor left_distributor;
    pond::Distributor right_distributor;
};

POND_MODULE_CPP_DECLARE(RealsenseDriver, "realsense_driver", "driver for the intel realsense d435")

pond_result RealsenseDriver::onStartup()
{
    POND_LOG("activating ...");

    color_distributor = createDistributor("color");
    left_distributor = createDistributor("left");
    right_distributor = createDistributor("right");
    
    uint32_t color_width = *parameter("color.width").getInt(1280);
    uint32_t color_height = *parameter("color.height").getInt(720);
    uint32_t mono_width = *parameter("mono.width").getInt(640);
    uint32_t mono_height = *parameter("mono.height").getInt(480);
    uint32_t fps = *parameter("fps").getInt(30);

    cfg.enable_stream(
        RS2_STREAM_INFRARED,
        1,
        mono_width,
        mono_height,
        RS2_FORMAT_Y8,
        fps
    );

    cfg.enable_stream(
        RS2_STREAM_INFRARED,
        2,
        mono_width,
        mono_height,
        RS2_FORMAT_Y8,
        fps
    );

    cfg.enable_stream(
        RS2_STREAM_COLOR, 
        color_width, 
        color_height, 
        RS2_FORMAT_RGB8,
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
    POND_LOG("deactivating ...");
    pipe.stop();
    color_distributor.destroy();
    left_distributor.destroy();
    right_distributor.destroy();
    POND_LOG("deactivated");
}

void RealsenseDriver::onFrame()
{
    rs2::frameset frames = pipe.wait_for_frames();
    rs2::video_frame color_frame = frames.get_color_frame();
    rs2::video_frame left_frame = frames.get_infrared_frame(1);
    rs2::video_frame right_frame = frames.get_infrared_frame(2);


    //distributor
    auto color_msg = std::make_shared<RealSenseWrappedImageFrame>(color_frame, "rgb8");
    color_distributor.distribute(static_cast<void*>(&color_msg));

    auto left_msg = std::make_shared<RealSenseWrappedImageFrame>(left_frame, "mono8");
    left_distributor.distribute(static_cast<void*>(&left_msg));

    auto right_msg = std::make_shared<RealSenseWrappedImageFrame>(right_frame, "mono8");
    right_distributor.distribute(static_cast<void*>(&right_msg));
}