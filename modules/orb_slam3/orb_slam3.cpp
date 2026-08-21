#define POND_MODULE_CPP_MAKE_IMPLEMENTATION
#include <chrono>
#include <pond/pond.hpp>
#include <pond/data_types/cv_img_frame.hpp>
#include <pond/data_types/imu_types.hpp>

#include "System.h"
#include "pond/data_types/video_types.hpp"
#include <mutex>

class OrbSlam3 : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
    std::shared_ptr<ORB_SLAM3::System> slam;

    std::string frame_id;
    ORB_SLAM3::System::eSensor mode;
    bool use_imu;

    std::mutex input_mutex;
    std::vector<ORB_SLAM3::IMU::Point> imu_data_points;
    std::atomic<bool> new_data;

    void store_imu_data(std::vector<ImuDataStamped>* imu_data)
    {
        imu_data_points.reserve((*imu_data).size());
        for (auto& d : *imu_data)
        {
            imu_data_points.push_back(ORB_SLAM3::IMU::Point(
                d.data.lin_acc.x, d.data.lin_acc.y, d.data.lin_acc.z,
                d.data.ang_vel.x, d.data.ang_vel.y, d.data.ang_vel.z,
                d.stamp.hw_time
            ));
        }
    }

    struct {
        bool use;
        pond::Receiver<ImgFrameSPtr, ImgFrameSPtr, std::vector<ImuDataStamped>> receiver_imu;
        pond::Receiver<ImgFrameSPtr, ImgFrameSPtr> receiver;
    } stereo;
    struct {
        bool use;
        pond::Receiver<ImgFrameSPtr, ImgFrameSPtr, std::vector<ImuDataStamped>> receiver_imu;
        pond::Receiver<ImgFrameSPtr, ImgFrameSPtr> receiver;
    } rgbd;
    struct {
        bool use;
        pond::Receiver<ImgFrameSPtr, std::vector<ImuDataStamped>> receiver_imu;
        pond::Receiver<ImgFrameSPtr> receiver;
    } mono;

    ImgFrameSPtr first_frame, second_frame;
    pond::Distributor<PoseStamped> pose_distributor;
    PoseStamped pose;
    pond::Distributor<ImgFrameSPtr> keypoint_frame_distributor;

    void distribute_result_pose(const Sophus::SE3f& Tcw)
    {
        if(Tcw.matrix().isZero()) return;
    
        Sophus::SE3f Twc = Tcw.inverse();

        Eigen::Vector3f t = Twc.translation();
        Eigen::Matrix3f R = Twc.rotationMatrix();
        
        pose.stamp.frame_id = "base_link";

        pose.pose.p.y = -t.x();
        pose.pose.p.z = -t.y();
        pose.pose.p.x = t.z();
        pose.pose.p.w = 1;

        pose.pose.o.y = -std::atan2(R(2,1), R(2,2));
        pose.pose.o.z = -std::atan2(
            -R(2,0),
            std::sqrt(R(2,1) * R(2,1) + R(2,2) * R(2,2))
        );
        pose.pose.o.x = std::atan2(R(1,0), R(0,0));
        pose.pose.o.w = 1;

        pose_distributor.distribute(pose);
    }

    void image_imu_callback(ImgFrameSPtr* first, ImgFrameSPtr* second, std::vector<ImuDataStamped>* imu_data)
    {
        std::lock_guard<std::mutex> lock(input_mutex);

        if (stereo.use) if ((*first)->format != ImgFrame::Format::Mono8 || (*second)->format != ImgFrame::Format::Mono8)
        {
            POND_LOG("ERROR: left->format (%s) != Mono8 || right->format (%s) != Mono8", ImgFrame::formatToString((*first)->format).c_str(), ImgFrame::formatToString((*second)->format).c_str());
            return;
        }
        if (rgbd.use) if ((*first)->format != ImgFrame::Format::RGB8 || (*second)->format != ImgFrame::Format::Depth16)
        {
            POND_LOG("ERROR: rgb->format (%s) != RGB8 || depth->format (%s) != Depth16", ImgFrame::formatToString((*first)->format).c_str(), ImgFrame::formatToString((*second)->format).c_str());
            return;
        }
        if (mono.use)if ((*first)->format != ImgFrame::Format::Mono8)
        {
            POND_LOG("ERROR: frame->format (%s) != Mono8", ImgFrame::formatToString((*first)->format).c_str());
            return;
        }
        
        new_data.store(true);

        first_frame = *first;
        if (!mono.use) second_frame = *second;
        if (use_imu) store_imu_data(imu_data);
    }
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
    
    use_imu = parameter("use_imu").asBool().get(false);
    frame_id = parameter("frame_id").asString().get("base_link");

    pose_distributor = createDistributor<PoseStamped>({"pose"});
    keypoint_frame_distributor = createDistributor<ImgFrameSPtr>({stereo.use ? "mono_left_with_keypoints/image" : (rgbd.use ? "color_with_keypoints/image" : "mono_with_keypoints/image")});

    new_data.store(false);

    slam = std::make_shared<ORB_SLAM3::System>(
        *vocabulary_path,
        *settings_path,
        ORB_SLAM3::System::STEREO,
        false
    );

    if (stereo.use)
    {
        if (use_imu) stereo.receiver_imu = createReceiver<ImgFrameSPtr, ImgFrameSPtr, std::vector<ImuDataStamped>>(
            {"mono_left/image", "mono_right/image", "imu_data"},
            [this](ImgFrameSPtr* left, ImgFrameSPtr* right, std::vector<ImuDataStamped>* imu_data)
            {
                image_imu_callback(left, right, imu_data);
            }
        );
        else stereo.receiver = createReceiver<ImgFrameSPtr, ImgFrameSPtr>(
            {"mono_left/image", "mono_right/image"},
            [this](ImgFrameSPtr* left, ImgFrameSPtr* right)
            {
                image_imu_callback(left, right, NULL);
            }
        );
    }
    if (rgbd.use)
    {
        if (use_imu) rgbd.receiver_imu = createReceiver<ImgFrameSPtr, ImgFrameSPtr, std::vector<ImuDataStamped>>(
            {"color/image", "depth/image", "imu_data"},
            [this](ImgFrameSPtr* color, ImgFrameSPtr* depth, std::vector<ImuDataStamped>* imu_data)
            {
                image_imu_callback(color, depth, imu_data);
            }
        );
        else rgbd.receiver = createReceiver<ImgFrameSPtr, ImgFrameSPtr>(
            {"color/image", "depth/image"},
            [this](ImgFrameSPtr* color, ImgFrameSPtr* depth)
            {
                image_imu_callback(color, depth, NULL);
            }
        );
    }
    if (mono.use)
    {
        if (use_imu) mono.receiver_imu = createReceiver<ImgFrameSPtr, std::vector<ImuDataStamped>>(
            {"mono/image", "imu_data"},
            [this](ImgFrameSPtr* frame, std::vector<ImuDataStamped>* imu_data)
            {
                image_imu_callback(frame, NULL, imu_data);
            }
        );
        else mono.receiver = createReceiver<ImgFrameSPtr>(
            {"mono/image"},
            [this](ImgFrameSPtr* frame)
            {
                image_imu_callback(frame, NULL, NULL);
            }
        );
    }

    return POND_SUCCESS;
}

void OrbSlam3::onShutdown()
{
    slam->Shutdown();
    slam.reset();

    if (stereo.use)
    {
        if (use_imu) stereo.receiver_imu.destroy();
        else stereo.receiver.destroy();
    }
    if (rgbd.use)
    {
        if (use_imu) rgbd.receiver_imu.destroy();
        else rgbd.receiver.destroy();
    }
    if (mono.use)
    {
        if (use_imu) mono.receiver_imu.destroy();
        else mono.receiver.destroy();
    }

    pose_distributor.destroy();
    keypoint_frame_distributor.destroy();
}

void OrbSlam3::onFrame()
{
    ImgFrameSPtr first_frame_c, second_frame_c;
    std::vector<ORB_SLAM3::IMU::Point> imu_data_points_c;

    if (new_data.load())
    {
        std::lock_guard<std::mutex> lock(input_mutex);
        new_data.store(false);

        first_frame_c = std::move(first_frame);
        if (!mono.use) second_frame_c = std::move(second_frame);
        if (use_imu) imu_data_points_c = std::move(imu_data_points);
    }
    else
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        return;
    }

    cv::Mat no_keypoint_mat;

    if (stereo.use)
    {
        cv::Mat left(first_frame_c->height, first_frame_c->width, CV_8UC1, first_frame_c->data);
        cv::Mat right(second_frame_c->height, second_frame_c->width, CV_8UC1, second_frame_c->data);

        pose.stamp.hw_time = (first_frame_c->stamp.hw_time + second_frame_c->stamp.hw_time) / 2.0;
        pose.stamp.time = (first_frame_c->stamp.time + second_frame_c->stamp.time) / 2.0;

        distribute_result_pose(slam->TrackStereo(left, right, pose.stamp.hw_time, imu_data_points_c));
        no_keypoint_mat = left;
    }
    if (rgbd.use)
    {
        cv::Mat rgb(first_frame_c->height, first_frame_c->width, CV_8UC3, first_frame_c->data);
        cv::Mat depth(second_frame_c->height, second_frame_c->width, CV_8UC2, second_frame_c->data);
        cv::Mat depth32;
        depth.convertTo(depth32, CV_32F, second_frame_c->depth_scale);

        pose.stamp.hw_time = (first_frame_c->stamp.hw_time + second_frame_c->stamp.hw_time) / 2.0;
        pose.stamp.time = (first_frame_c->stamp.time + second_frame_c->stamp.time) / 2.0;
        distribute_result_pose(slam->TrackRGBD(rgb, depth32, pose.stamp.hw_time, imu_data_points_c));
        no_keypoint_mat = rgb;
    }
    if (mono.use)
    {
        cv::Mat mono(first_frame_c->height, first_frame_c->width, CV_8UC1, first_frame_c->data);

        pose.stamp.hw_time = first_frame_c->stamp.hw_time;
        pose.stamp.time = first_frame_c->stamp.time;
        distribute_result_pose(slam->TrackMonocular(mono, pose.stamp.hw_time, imu_data_points_c));
        no_keypoint_mat = mono;
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
