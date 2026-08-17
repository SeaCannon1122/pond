#include <chrono>
#include <thread>
#include <memory>
#include <pond/pond.hpp>
#include <opencv2/opencv.hpp>
#include <pond/data_types.hpp>
#include <vector>
#include <random>


std::vector<uint8_t> makeDummyImage(std::size_t w, std::size_t h)
{
    using namespace std::chrono;

    const auto now = steady_clock::now();
    const auto t = duration_cast<milliseconds>(
        now.time_since_epoch()
    ).count();

    std::vector<uint8_t> image(w * h * 3);

    for (std::size_t y = 0; y < h; ++y)
    {
        for (std::size_t x = 0; x < w; ++x)
        {
            const std::size_t i = (y * w + x) * 3;

            image[i + 0] = static_cast<uint8_t>((x + t / 10) % 256);
            image[i + 1] = static_cast<uint8_t>((y + t / 15) % 256);
            image[i + 2] = static_cast<uint8_t>((x + y + t / 20) % 256);
        }
    }

    return image;
}

class DummyImgFrame : public ImgFrame
{
public:

    explicit DummyImgFrame(uint32_t w, uint32_t h, const std::string& format_)
    {
        image_data = makeDummyImage(w, h);
        data = image_data.data();
        width = w;
        height = h;
        pixel_size = 3;
        format = format_;  
    }

private:
    std::vector<uint8_t> image_data;
};

class DummyCamera : public pond::ModuleBase
{
public:
    virtual pond_result onStartup() override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
    pond::Distributor<ImgFrameSPtr> distributor;
    uint32_t width, height, fps;
    std::chrono::steady_clock::time_point last_time;
};

POND_MODULE_CPP_DECLARE(DummyCamera, "dummy_camera", "distributing dummy images for testing")

pond_result DummyCamera::onStartup()
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

    ImgFrameSPtr color_msg = std::make_shared<DummyImgFrame>(width, height, "rgb8");
    distributor.distribute(color_msg);
}
