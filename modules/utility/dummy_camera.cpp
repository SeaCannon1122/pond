#include <pond/pond.hpp>
#include <pond_data_types/video_types.hpp>
#include <thread>

class DummyCamera : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
    pond::DistributorTyped<ImgFrameSPtr> distributor;
    uint32_t width, height, fps;
    double last_time = 0;
    std::string frame_id;
};

POND_MODULE_CPP_DECLARE(DummyCamera, "dummy_camera", "distributing dummy images for testing")

pond_result DummyCamera::onStartup(const std::vector<void*>& args)
{
    distributor = createDistributorTyped<ImgFrameSPtr>({"color/image"});
    width = parameter("width").asInt().get(640);
    height = parameter("height").asInt().get(480);
    fps = parameter("fps").asInt().get(30);
    frame_id = parameter("frame_id").asString().get("camera_frame");

    return POND_SUCCESS;
}

void DummyCamera::onShutdown()
{
    distributor.destroy();
}

void DummyCamera::onFrame()
{
    double this_time = pond::get_time();
    if (last_time != 0.0)
    {
        double ft = 1.0/(double)fps;
        double remaining = ft - (this_time - last_time);

        if (remaining > 0.0005)
        {
            std::this_thread::sleep_for(std::chrono::duration<double>(remaining));
            last_time += ft; 
        }
        else last_time = this_time;
    }
    else last_time = this_time;

    size_t t = this_time * 1000.0;
    
    ImgFrameSPtr color_msg = std::make_shared<ImgFrame>();
    color_msg->default_data_buffer.resize(3*width*height);

    for (std::size_t y = 0; y < height; y++)
    {
        for (std::size_t x = 0; x < width; x++)
        {
            size_t i = (y * width + x) * 3;

            color_msg->default_data_buffer[i + 0] = static_cast<uint8_t>((x + y + t / 20) % 256);
            color_msg->default_data_buffer[i + 2] = static_cast<uint8_t>((x + t / 10) % 256);
            color_msg->default_data_buffer[i + 1] = static_cast<uint8_t>((y + t / 15) % 256);
        }
    }

    color_msg->data = color_msg->default_data_buffer.data();
    color_msg->width = width;
    color_msg->height = height;
    color_msg->pixel_size = 3;
    color_msg->format = ImgFrame::Format::RGB8;  
    color_msg->stamp.frame_id = frame_id;
    color_msg->stamp.time = this_time;
    color_msg->stamp.hw_time = this_time;

    distributor.distribute(&color_msg);
}
