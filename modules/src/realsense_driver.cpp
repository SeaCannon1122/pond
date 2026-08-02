#include <pond/pond.hpp>
#include <librealsense2/rs.hpp>
#include "wrapped_image_frame.hpp"
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

    RealSenseWrappedImageFrame* clone() const override
    {
        return new RealSenseWrappedImageFrame(*this);
    }

private:
    rs2::video_frame frame;
};

class RealsenseDriver : public pond::ModuleBase
{
public:
    virtual pond_result onActivate() override;
    virtual void onDeactivate() override;
    virtual void onFrame() override;
private:
    rs2::pipeline pipe;
    rs2::config cfg;
    pond::Distributor color_distributor;
    pond::Distributor left_distributor;
    pond::Distributor right_distributor;
};

POND_MODULE_CPP_DECLARE(RealsenseDriver, "realsense_driver", "driver for the intel realsense d435")

pond_result RealsenseDriver::onActivate()
{
    POND_LOG("activating ...");

    color_distributor = createDistributor("color");
    left_distributor = createDistributor("left");
    right_distributor = createDistributor("right");
    
    cfg.enable_stream(
        RS2_STREAM_INFRARED,
        1,
        640,
        480,
        RS2_FORMAT_Y8,
        30
    );

    cfg.enable_stream(
        RS2_STREAM_INFRARED,
        2,
        640,
        480,
        RS2_FORMAT_Y8,
        30
    );

    cfg.enable_stream(
        RS2_STREAM_COLOR, 
        1280, 
        720, 
        RS2_FORMAT_RGB8,
        30
    );        
        
    rs2::pipeline_profile profile = pipe.start(cfg);

    for (auto&& sensor : profile.get_device().query_sensors())
    {
        if (sensor.supports(RS2_OPTION_EMITTER_ENABLED)) sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0);
    }

    POND_LOG("activated");
    return POND_SUCCESS;
}

void RealsenseDriver::onDeactivate()
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
    auto color_msg = RealSenseWrappedImageFrame(color_frame, "rgb8");
    color_distributor.distribute(static_cast<void*>(&color_msg));

    auto* left_msg = new std::shared_ptr<WrappedImageFrame>(std::make_shared<RealSenseWrappedImageFrame>(left_frame, "mono8"));
    left_distributor.distribute(static_cast<void*>(left_msg));

    auto* right_msg = new std::shared_ptr<WrappedImageFrame>(std::make_shared<RealSenseWrappedImageFrame>(right_frame, "mono8"));
    right_distributor.distribute(static_cast<void*>(right_msg));
}