#include "pond/data_types/transform_types.hpp"
#define POND_MODULE_CPP_MAKE_IMPLEMENTATION

#include <pond/pond.hpp>
#include <pond/data_types/cv_img_frame.hpp>
#include <pond/data_types/imu_types.hpp>
#include <pond/data_types/transform_types.hpp>

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

    ORB_SLAM3::System::eSensor mode;

    std::mutex frame_mutex;
    std::mutex imu_mutex;
    std::vector<ORB_SLAM3::IMU::Point> imu_data_points;
    std::atomic<bool> new_frame;

    struct {
        bool use;
        pond::Receiver<ImgFrameSPtr, ImgFrameSPtr> receiver;
    } stereo;
    struct {
        bool use;
        pond::Receiver<ImgFrameSPtr, ImgFrameSPtr> receiver;
    } rgbd;
    struct {
        bool use;
        pond::Receiver<ImgFrameSPtr> receiver;
    } mono;

    pond::Receiver<ImuData> imu_receiver;

    ImgFrameSPtr first_frame, second_frame;
    pond::Distributor<FrameTransform> transform_distributor;
    FrameTransform transform;
    pond::Distributor<ImgFrameSPtr> keypoint_frame_distributor;
};

POND_MODULE_CPP_DECLARE(OrbSlam3, "slam", "Supports mono, stereo and rgbd vslam")

POND_BUNDLE_DECLARE(
    "ORB_SLAM3 pond bundle", 
    1,
    POND_MODULE(OrbSlam3),
)

pond_result OrbSlam3::onStartup(const std::vector<void*>& args)
{
    auto vocabulary_path = parameter("vocabulary_path").asString().getStrict();
    auto settings_path = parameter("camera_info_path").asString().getStrict();
    auto slam_mode = parameter("mode").asString().getStrict({"Stereo", "RGBD", "Mono"});
    if (!vocabulary_path || !settings_path || !slam_mode) return POND_ERROR;
    
    if (slam_mode == "Stereo") stereo.use = true;
    else stereo.use = false;
    if (slam_mode == "RGBD") rgbd.use = true;
    else rgbd.use = false;
    if (slam_mode == "Mono") mono.use = true;
    else mono.use = false;
    
    transform.stamp.frame_id = parameter("base_frame_id").asString().get("world");
    transform.child_frame_id = parameter("camera_frame_id").asString().get("camera");

    transform_distributor = createDistributor<FrameTransform>({"slam_transform"});
    keypoint_frame_distributor = createDistributor<ImgFrameSPtr>({stereo.use ? "mono_left_with_keypoints/image" : (rgbd.use ? "color_with_keypoints/image" : "mono_with_keypoints/image")});

    new_frame.store(false);

    slam = std::make_shared<ORB_SLAM3::System>(
        *vocabulary_path,
        *settings_path,
        ORB_SLAM3::System::STEREO,
        false
    );

    if (stereo.use) stereo.receiver = createReceiver<ImgFrameSPtr, ImgFrameSPtr>(
        {"stereo_left/image", "stereo_right/image"},
        [this](ImgFrameSPtr* left, ImgFrameSPtr* right)
        {
            std::lock_guard<std::mutex> lock(frame_mutex);

            if ((*left)->format != ImgFrame::Format::Mono8 || (*right)->format != ImgFrame::Format::Mono8)
            {
                POND_LOG("ERROR: left->format (%s) != Mono8 || right->format (%s) != Mono8", ImgFrame::formatToString((*left)->format).c_str(), ImgFrame::formatToString((*right)->format).c_str());
                return;
            }
            new_frame.store(true);
            first_frame = *left; second_frame = *right;
        }
    );
    if (rgbd.use) rgbd.receiver = createReceiver<ImgFrameSPtr, ImgFrameSPtr>(
        {"color/image", "depth/image"},
        [this](ImgFrameSPtr* color, ImgFrameSPtr* depth)
        {
            std::lock_guard<std::mutex> lock(frame_mutex);

            if ((*color)->format != ImgFrame::Format::RGB8 || (*depth)->format != ImgFrame::Format::Depth16)
            {
                POND_LOG("ERROR: rgb->format (%s) != RGB8 || depth->format (%s) != Depth16", ImgFrame::formatToString((*color)->format).c_str(), ImgFrame::formatToString((*depth)->format).c_str());
                return;
            }
            new_frame.store(true);
            first_frame = *color; second_frame = *depth;
        }
    );
    if (mono.use) mono.receiver = createReceiver<ImgFrameSPtr>(
        {"mono/image"},
        [this](ImgFrameSPtr* frame)
        {
            std::lock_guard<std::mutex> lock(frame_mutex);

            if ((*frame)->format != ImgFrame::Format::Mono8)
            {
                POND_LOG("ERROR: frame->format (%s) != Mono8", ImgFrame::formatToString((*frame)->format).c_str());
                return;
            }
            new_frame.store(true);
            first_frame = *frame;
        }
    );

    imu_data_points.reserve(100);
    imu_receiver = createReceiver<ImuData>({"imu"}, [this](ImuData* data){
    
        std::lock_guard<std::mutex> lock(imu_mutex);

        imu_data_points.push_back(ORB_SLAM3::IMU::Point(
            data->lin_acc[0], data->lin_acc[1], data->lin_acc[2],
            data->ang_vel[0], data->ang_vel[1], data->ang_vel[2],
            data->stamp.hw_time
        ));
    });

    return POND_SUCCESS;
}

void OrbSlam3::onShutdown()
{
    slam->Shutdown();
    slam.reset();

    if (stereo.use) stereo.receiver.destroy();
    if (rgbd.use) rgbd.receiver.destroy();
    if (mono.use) mono.receiver.destroy();

    imu_receiver.destroy();

    transform_distributor.destroy();
    keypoint_frame_distributor.destroy();
}

void OrbSlam3::onFrame()
{
    ImgFrameSPtr first_frame_c, second_frame_c;
    std::vector<ORB_SLAM3::IMU::Point> imu_data_points_c;

    if (new_frame.load())
    {
        std::lock_guard<std::mutex> lock(frame_mutex);
        new_frame.store(false);

        first_frame_c = std::move(first_frame);
        if (!mono.use) second_frame_c = std::move(second_frame);
    }
    else
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        return;
    }

    {
        std::lock_guard<std::mutex> lock(imu_mutex);
        imu_data_points_c = std::move(imu_data_points);
        imu_data_points.clear();
        imu_data_points.reserve(100);
    }

    cv::Mat no_keypoint_mat;
    Sophus::SE3f Tcw;

    if (stereo.use)
    {
        cv::Mat left(first_frame_c->height, first_frame_c->width, CV_8UC1, first_frame_c->data);
        cv::Mat right(second_frame_c->height, second_frame_c->width, CV_8UC1, second_frame_c->data);

        transform.stamp.hw_time = (first_frame_c->stamp.hw_time + second_frame_c->stamp.hw_time) / 2.0;
        transform.stamp.time = (first_frame_c->stamp.time + second_frame_c->stamp.time) / 2.0;

        Tcw = slam->TrackStereo(left, right, transform.stamp.hw_time, imu_data_points_c);
        no_keypoint_mat = left;
    }
    if (rgbd.use)
    {
        cv::Mat rgb(first_frame_c->height, first_frame_c->width, CV_8UC3, first_frame_c->data);
        cv::Mat depth(second_frame_c->height, second_frame_c->width, CV_8UC2, second_frame_c->data);
        cv::Mat depth32;
        depth.convertTo(depth32, CV_32F, second_frame_c->depth_scale);

        transform.stamp.hw_time = (first_frame_c->stamp.hw_time + second_frame_c->stamp.hw_time) / 2.0;
        transform.stamp.time = (first_frame_c->stamp.time + second_frame_c->stamp.time) / 2.0;
        Tcw = slam->TrackRGBD(rgb, depth32, transform.stamp.hw_time, imu_data_points_c);
        no_keypoint_mat = rgb;
    }
    if (mono.use)
    {
        cv::Mat mono(first_frame_c->height, first_frame_c->width, CV_8UC1, first_frame_c->data);

        transform.stamp.hw_time = first_frame_c->stamp.hw_time;
        transform.stamp.time = first_frame_c->stamp.time;
        Tcw = slam->TrackMonocular(mono, transform.stamp.hw_time, imu_data_points_c);
        no_keypoint_mat = mono;
    }

    if(!Tcw.matrix().isZero())
    {
        transform.tf = Tcw.cast<double>().inverse();
        transform_distributor.distribute(transform);
    }

    std::vector<cv::KeyPoint> keypoints = slam->GetTrackedKeyPointsUn();

    cv::Mat keypoint_mat;
    cv::drawKeypoints(
        no_keypoint_mat,
        keypoints,
        keypoint_mat,
        cv::Scalar(0, 255, 0)
    );

    ImgFrameSPtr keypoint_frame = std::make_shared<CVImgFrame>(keypoint_mat, rgbd.use ? ImgFrame::Format::RGB8 : ImgFrame::Format::BGR8);
    keypoint_frame_distributor.distribute(keypoint_frame);
}
