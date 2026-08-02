#include "realsense_driver.hpp"
#include <memory>
#include <type_traits>

pond_result RealsenseDriver::onActivate()
{
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
}

void RealsenseDriver::onDeactivate()
{
    pipe.stop();
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

POND_MODULE_CPP_DECLARE(RealsenseDriver)