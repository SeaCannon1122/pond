#include <pond/pond.hpp>
#include <chrono>
#include <thread>

class FrameTimer : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
    double min_time;
    double last_time = 0;
};

POND_MODULE_CPP_DECLARE(FrameTimer, "frame_timer", "times frames to a minimum fps")

pond_result FrameTimer::onStartup(const std::vector<void*>& args)
{
    auto min_time_o = parameter("min_time").asDouble().getStrict();
    if (!min_time_o) return POND_ERROR;

    min_time = *min_time_o;
    if (min_time < 0.0 || min_time > 10.0)
    {
        POND_LOG("Error when checking 0.0 <= min_time (%f) <= 10.0", min_time);
        return POND_ERROR;
    }

    return POND_SUCCESS;
}

void FrameTimer::onShutdown()
{
}

void FrameTimer::onFrame()
{
    if (last_time != 0.0)
    {
        double this_time = pond::get_time();
        double remaining = min_time - (this_time - last_time);

        if (remaining > 0.0)
        {
            std::this_thread::sleep_for(std::chrono::duration<double>(remaining));
            last_time += min_time; 
        }
        else last_time = this_time;
    }
    else last_time = pond::get_time();
}
