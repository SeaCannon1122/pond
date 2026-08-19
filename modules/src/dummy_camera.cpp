#include <pond/pond.hpp>
#include <pond/data_types/data_types.hpp>
#include <thread>

class DummyCamera : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
    pond::Distributor<ImgFrameSPtr> distributor;
    uint32_t width, height, fps;
    std::chrono::steady_clock::time_point last_time;
};

POND_MODULE_CPP_DECLARE(DummyCamera, "dummy_camera", "distributing dummy images for testing")

pond_result DummyCamera::onStartup(const std::vector<void*>& args)
{
    distributor = createDistributor<ImgFrameSPtr>({"out"});
    width = parameter("width").asInt().get(640);
    height = parameter("height").asInt().get(480);
    fps = parameter("fps").asInt().get(30);

    last_time = std::chrono::steady_clock::now();
    return POND_SUCCESS;
}

void DummyCamera::onShutdown()
{
    distributor.destroy();
}

void DummyCamera::onFrame()
{
    auto remaining = std::chrono::duration<double>(1.0 / (double)fps) - (std::chrono::steady_clock::now() - last_time);
    if (remaining > std::chrono::duration<double>::zero()) std::this_thread::sleep_for(remaining);
    last_time = std::chrono::steady_clock::now();

    ImgFrameSPtr color_msg = std::make_shared<ImgFrame>();

    auto now = std::chrono::steady_clock::now();
    auto t = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

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

    distributor.distribute(color_msg);
}
