#include <memory>
#include <mutex>
#include <pond/pond.hpp>
#include "System.h"
#include "pond/pond.h"
#include <quac_modules/interfaces/wrapped_image_frame.hpp>

class OrbSlam3Module : public pond::ModuleBase
{
public:
    virtual pond_result onStartup() override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
    std::shared_ptr<ORB_SLAM3::System> slam;
    pond::Receiver left_receiver;
    pond::Receiver right_receiver;
    bool use_imu;

    std::mutex input_mutex;
    std::optional<std::shared_ptr<WrappedImageFrame>> left_frame;

    std::optional<std::shared_ptr<WrappedImageFrame>> right_frame;
};

POND_MODULE_CPP_DECLARE(OrbSlam3Module, "orb_slam3", "pond module for orbslam3")

pond_result OrbSlam3Module::onStartup()
{
    auto vocabulary_path = parameter("vocabulary_path").getString("", false);
    auto settings_path = parameter("camera_info_path").getString("", false);
    if (!vocabulary_path || !settings_path) return POND_ERROR;

    use_imu = *parameter("use_imu").getBool("false");

    slam = std::make_shared<ORB_SLAM3::System>(
        *vocabulary_path,
        *settings_path,
        ORB_SLAM3::System::STEREO,
        false
    );

    left_receiver = createReceiver("left", [this](void* data) {
        std::lock_guard<std::mutex> lock(input_mutex);
        left_frame = *(std::shared_ptr<WrappedImageFrame>*)data;
    });

    return POND_SUCCESS;
}

void OrbSlam3Module::onShutdown()
{
    slam->Shutdown();
    slam.reset();
}

void OrbSlam3Module::onFrame()
{
    Sophus::SE3f Tcw = slam->TrackStereo(left, right, timestamp, imus);

    if(!Tcw.matrix().isZero())
    {
    }
}
