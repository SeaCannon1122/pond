
#include <pond/pond.hpp>
#include <pond/data_types/data_types.hpp>

#include "System.h"
#include <mutex>

class OrbSlam3 : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
    std::shared_ptr<ORB_SLAM3::System> slam;
    pond::Receiver<ImgFrameSPtr, ImgFrameSPtr, std::vector<ImuDataStamped>> receiver_with_imu;
    pond::Receiver<ImgFrameSPtr, ImgFrameSPtr> receiver_without_imu;
    bool use_imu;

    std::mutex input_mutex;
    ImgFrameSPtr left_frame, right_frame;
    std::vector<ORB_SLAM3::IMU::Point> imu_data_points;
    bool new_data;
};

POND_MODULE_CPP_DECLARE(OrbSlam3, "orb_slam3", "pond module for orbslam3")

pond_result OrbSlam3::onStartup(const std::vector<void*>& args)
{
    auto vocabulary_path = parameter("vocabulary_path").asString().getStrict();
    auto settings_path = parameter("camera_info_path").asString().getStrict();
    if (!vocabulary_path || !settings_path) return POND_ERROR;

    if (use_imu = parameter("use_imu").asBool().get(false))
    {
        receiver_with_imu = createReceiver<ImgFrameSPtr, ImgFrameSPtr, std::vector<ImuDataStamped>>(
            {"mono_left/image", "mono_right/image", "imu_data"},
            [this](ImgFrameSPtr& left, ImgFrameSPtr& right, std::vector<ImuDataStamped>& imu_data)
            {
                std::lock_guard<std::mutex> lock(input_mutex);

                if (left->format != ImgFrame::Format::Mono8 || right->format != ImgFrame::Format::Mono8)
                {
                    POND_LOG("ERROR: left->format (%s) != Mono8 || right->format (%s) != Mono8", ImgFrame::formatToString(left->format).c_str(), ImgFrame::formatToString(right->format).c_str());
                    return;
                }

                new_data = true;

                imu_data_points.reserve(imu_data.size());
                for (auto& d : imu_data)
                {
                    imu_data_points.push_back(ORB_SLAM3::IMU::Point(
                        d.data.lin_acc.x, d.data.lin_acc.y, d.data.lin_acc.z,
                        d.data.ang_vel.x, d.data.ang_vel.y, d.data.ang_vel.z,
                        d.stamp.hw_time
                    ));
                }

                left_frame = left;
                right_frame = right;
            }
        );
    }
    else
    {
        receiver_without_imu = createReceiver<ImgFrameSPtr, ImgFrameSPtr>(
            {"mono_left/image", "mono_right/image"},
            [this](ImgFrameSPtr& left, ImgFrameSPtr& right)
            {
                std::lock_guard<std::mutex> lock(input_mutex);

                if (left->format != ImgFrame::Format::Mono8 || right->format != ImgFrame::Format::Mono8)
                {
                    POND_LOG("ERROR: left->format (%s) != Mono8 || right->format (%s) != Mono8", ImgFrame::formatToString(left->format).c_str(), ImgFrame::formatToString(right->format).c_str());
                    return;
                }

                new_data = true;

                left_frame = left;
                right_frame = right;
            }
        );
    }

    new_data = false;

    slam = std::make_shared<ORB_SLAM3::System>(
        *vocabulary_path,
        *settings_path,
        ORB_SLAM3::System::STEREO,
        false
    );

    return POND_SUCCESS;
}

void OrbSlam3::onShutdown()
{
    slam->Shutdown();
    slam.reset();

    if (use_imu) receiver_with_imu.destroy();
    else receiver_without_imu.destroy();
}

void OrbSlam3::onFrame()
{
    ImgFrameSPtr left_frame_c, right_frame_c;
    std::vector<ORB_SLAM3::IMU::Point> imu_data_points_c;

    {
        std::lock_guard<std::mutex> lock(input_mutex);
        if (!new_data) return;
        left_frame_c = std::move(left_frame); right_frame_c = std::move(right_frame);
        if (use_imu) imu_data_points_c = std::move(imu_data_points);
    }

    cv::Mat left(left_frame_c->height, left_frame_c->width, CV_8UC1, left_frame_c->data);
    cv::Mat right(left_frame_c->height, left_frame_c->width, CV_8UC1, left_frame_c->data);

    Sophus::SE3f Tcw = slam->TrackStereo(left, right, (left_frame_c->stamp.hw_time + right_frame_c->stamp.hw_time) / 2, imu_data_points_c);

    if(!Tcw.matrix().isZero())
    {
        
    }
}
