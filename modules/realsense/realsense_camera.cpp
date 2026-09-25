#include <pond/pond.hpp>
#include <pond_data_types/video_types.hpp>

#include <librealsense2/rs.hpp>

class RealSenseImgFrame : public ImgFrame
{
public:

    explicit RealSenseImgFrame(const rs2::video_frame& frame_, ImgFrame::Format format_, const std::string& frame_id) : frame(frame_)
    {
        data = (void*)frame.get_data();
        width = frame.get_width();
        height = frame.get_height();
        pixel_size = frame.get_bytes_per_pixel();
        format = format_;
        if (format == ImgFrame::Format::Depth8 || format == ImgFrame::Format::Depth16) depth_scale = frame_.as<rs2::depth_frame>().get_units();

        stamp.hw_time = frame.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP);
        stamp.time = frame.get_frame_metadata(RS2_FRAME_METADATA_BACKEND_TIMESTAMP);
        stamp.frame_id = frame_id;
    }

private:
    rs2::video_frame frame;
};

class RealsenseCamera : public pond::ModuleBase
{
public:
    RealsenseCamera() : align_depth_to_color(RS2_STREAM_COLOR) {}
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:

    bool enqueue_frame(const rs2::video_frame& frame, CameraInfo& info, const char* frame_name);

    rs2::pipeline pipe;
    rs2::config cfg;

    bool mode_color;
    pond::Distributor distributor;
    bool mode_color_depth;
    bool align_depth;
    bool mode_color_stereo;

    CameraInfo color_info, depth_info, aligned_depth_info, stereo_left_info, stereo_right_info;
    
    rs2::align align_depth_to_color;
};

POND_MODULE_CPP_DECLARE(RealsenseCamera, "camera", "driver for the intel realsense d435")

POND_BUNDLE_DECLARE(
    "realsense pond bundle",
    POND_MODULE(RealsenseCamera),
)

pond_result RealsenseCamera::onStartup(const std::vector<void*>& args)
{
    std::string mode;
    auto mode_o = parameter("mode").asString().getStrict({"color", "color_depth", "color_stereo"});
    if (!mode_o) return POND_ERROR; else mode = *mode_o;
    mode_color = (mode == "color"); mode_color_depth = (mode == "color_depth"); mode_color_stereo = (mode == "color_stereo");

    std::vector<int32_t> color_dims = parameter("color.dims").asIntArray().get({1280, 720}, 2, 2);
    std::vector<int32_t> depth_dims = parameter("depth.dims").asIntArray().get({640, 480}, 2, 2);
    std::vector<int32_t> stereo_dims = parameter("stereo.dims").asIntArray().get({640, 480}, 2, 2);
    uint32_t fps = parameter("fps").asInt().get(30);

    color_info.stamp.frame_id = parameter("color.frame_id").asString().get("color_optical_frame");
    depth_info.stamp.frame_id = parameter("depth.frame_id").asString().get("depth_optical_frame");
    stereo_left_info.stamp.frame_id = parameter("stereo.left_frame_id").asString().get("stereo_left_optical_frame");
    stereo_right_info.stamp.frame_id = parameter("stereo.right_frame_id").asString().get("stereo_right_optical_frame");

    color_info.fps = fps; depth_info.fps = fps; aligned_depth_info.fps = fps; stereo_right_info.fps = fps; stereo_left_info.fps = fps;
    color_info.format = ImgFrame::Format::RGB8; depth_info.format = ImgFrame::Format::Depth16; aligned_depth_info.format = ImgFrame::Format::Depth16;
    stereo_right_info.format = ImgFrame::Format::Mono8; stereo_left_info.format = ImgFrame::Format::Mono8;

    align_depth = parameter("depth.align_to_color").asBool().get(false);
    aligned_depth_info.stamp.frame_id = color_info.stamp.frame_id;

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

        if (!connected) POND_LOG_RETURN_ERROR("No device with serial number '%s' connected", serial_number->c_str());
    }

    cfg.enable_stream(RS2_STREAM_COLOR, color_dims[0], color_dims[1], RS2_FORMAT_RGB8, fps);

    if (mode_color_depth) cfg.enable_stream(RS2_STREAM_DEPTH, depth_dims[0], depth_dims[1], RS2_FORMAT_Z16, fps);

    if (mode_color_stereo)
    {
        cfg.enable_stream(RS2_STREAM_INFRARED, 1, stereo_dims[0], stereo_dims[1], RS2_FORMAT_Y8, fps);
        cfg.enable_stream(RS2_STREAM_INFRARED, 2, stereo_dims[0], stereo_dims[1], RS2_FORMAT_Y8, fps);
    }
        
    rs2::pipeline_profile profile = pipe.start(cfg);

    for (auto&& sensor : profile.get_device().query_sensors())
        if (sensor.supports(RS2_OPTION_EMITTER_ENABLED))
            sensor.set_option(RS2_OPTION_EMITTER_ENABLED, mode_color_depth ? 1 : 0);

    if (mode_color) distributor = createDistributor<ImgFrameSPtr, CameraInfo>({"color/image", "color/info", });
    if (mode_color_depth)
    {
        if (align_depth) distributor = createDistributor<ImgFrameSPtr, CameraInfo, ImgFrameSPtr, CameraInfo, ImgFrameSPtr, CameraInfo>(
            {
                "color/image", "color/info", 
                "depth/image", "depth/info", 
                "depth_aligned/image", "depth_aligned/info", 
            }
        );
        else distributor = createDistributor<ImgFrameSPtr, CameraInfo, ImgFrameSPtr, CameraInfo>(
            {
                "color/image", "color/info", 
                "depth/image", "depth/info", 
            }
        );
    }
    if (mode_color_stereo) distributor = createDistributor<ImgFrameSPtr, CameraInfo, ImgFrameSPtr, CameraInfo, ImgFrameSPtr, CameraInfo>(
        {
            "color/image", "color/info", 
            "stereo_left/image", "stereo_left/info", 
            "stereo_right/image", "stereo_right/info"
        }
    );

    return POND_SUCCESS;
}

void RealsenseCamera::onShutdown()
{
    pipe.stop();
    distributor.destroy();
}

void set_camera_info(CameraInfo& info, const rs2::frame& frame)
{
    rs2_intrinsics intrinsics = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    info.width = intrinsics.width;
    info.height = intrinsics.height;

    info.k.setZero();
    info.k(0, 0) = intrinsics.fx;
    info.k(0, 2) = intrinsics.ppx;
    info.k(1, 1) = intrinsics.fy;
    info.k(1, 2) = intrinsics.ppy;
    info.k(2, 2) = 1.0;

    if (intrinsics.model == RS2_DISTORTION_KANNALA_BRANDT4)
    {
        info.distortion_model = "equidistant";
        info.d.resize(4);
    }
    else
    {
        info.distortion_model = "plumb_bob";
        info.d.resize(5);
    }

    for (int i = 0; i < info.d.size(); i++) info.d[i] = intrinsics.coeffs[i];
}

bool RealsenseCamera::enqueue_frame(const rs2::video_frame& frame, CameraInfo& info, const char* frame_name)
{
    if (!frame)
    {
        POND_LOG("%s frame is empty", frame_name);
        return false;
    }

    ImgFrameSPtr msg = std::make_shared<RealSenseImgFrame>(frame, info.format, info.stamp.frame_id);

    if (info.stamp.time == 0) set_camera_info(info, frame);
    info.stamp.time = msg->stamp.time;
    info.stamp.hw_time = msg->stamp.hw_time;

    distributor.enqueue(&msg, &info);

    return true;
}

void RealsenseCamera::onFrame()
{
    distributor.clear_queue();

    rs2::frameset frames = pipe.wait_for_frames();

    if (!enqueue_frame(frames.get_color_frame(), color_info, "color")) return;

    if (mode_color_depth) if (!enqueue_frame(frames.get_depth_frame(), depth_info, "depth")) return;
    if (mode_color_depth && align_depth) if (!enqueue_frame(align_depth_to_color.process(frames).get_depth_frame(), aligned_depth_info, "aligned depth")) return;

    if (mode_color_stereo) if (!enqueue_frame(frames.get_infrared_frame(1), stereo_left_info, "stereo left")) return;
    if (mode_color_stereo) if (!enqueue_frame(frames.get_infrared_frame(2), stereo_right_info, "stereo right")) return;

    distributor.distribute_enqueued();
}