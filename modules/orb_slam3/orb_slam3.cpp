#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>
#include <pond_data_types/cv_img_frame.hpp>
#include <pond_data_types/imu_types.hpp>
#include <sophus/se3.hpp>

#include "System.h"
#include "pond/pond.h"
#include "pond_data_types/robot_state_types.hpp"

#define SETTINGS_PATH "~/.cache/orbslam_config.yaml"

class OrbSlam3 : public pond::ModuleBaseTF
{
public:
    virtual pond_result onStartupTF(const std::vector<void*>& args) override;
    virtual void onShutdownTF() override;
    virtual void onFrame() override;
private:

    bool get_cam_info(CameraInfo& cam_info, const std::string& channel);
    bool get_imu_info(ImuInfo& info);
    bool create_config(CameraInfo& cam_info);

    std::shared_ptr<ORB_SLAM3::System> slam;
    std::string vocabulary_path, base_link_frame_id;

    int32_t n_features, n_levels, ini_th_fast, min_th_fast; double scale_factor, far_threshold; bool imu_insert_kfs_when_lost;

    std::mutex frame_mutex, imu_mutex;
    std::vector<ORB_SLAM3::IMU::Point> imu_data_points;
    ImgFrameSPtr first_frame, second_frame;
    std::atomic<bool> new_frame;

    bool use_imu, stereo_mode, rgbd_mode, mono_mode;

    pond::Receiver image_receiver, imu_receiver;

    pond::DistributorTyped<FrameTransform> transform_distributor;
    FrameTransform transform; Sophus::SE3d cam_tf_start;
    pond::DistributorTyped<ImgFrameSPtr> keypoint_frame_distributor;
};

POND_MODULE_CPP_DECLARE(OrbSlam3, "slam", "Supports mono, stereo and rgbd vslam")

POND_BUNDLE_DECLARE("ORB_SLAM3 pond bundle",POND_MODULE(OrbSlam3))

bool OrbSlam3::get_cam_info(CameraInfo& info, const std::string& channel)
{
    pond::Receiver info_receiver = createReceiver<CameraInfo>({channel}, [&](CameraInfo* info_msg) {if (info.stamp.time == 0) info = *info_msg;});
    for (int i = 0; i < 100 && info.stamp.time == 0; i++) pond::sleep(0.01);
    info_receiver.destroy();
    if (info.stamp.time == 0) POND_LOG_RETURN_FALSE("Did not receive camera info on channel '%s'", channel.c_str());
    return true;
}

bool OrbSlam3::get_imu_info(ImuInfo& info)
{
    pond::Receiver info_receiver = createReceiver<ImuInfo>({"imu/info"}, [&](ImuInfo* info_msg) {if (info.stamp.time == 0) info = *info_msg;});
    for (int i = 0; i < 100 && info.stamp.time == 0; i++) pond::sleep(0.01);
    info_receiver.destroy();
    if (info.stamp.time == 0) POND_LOG_RETURN_FALSE("Did not receive imu info on channel 'imu/info'");
    return true;
}

pond_result OrbSlam3::onStartupTF(const std::vector<void*>& args)
{
    n_features = parameter("ORBextractor.nFeatures").asInt().get(1250);
    scale_factor = parameter("ORBextractor.scaleFactor").asDouble().get(1.2);
    n_levels = parameter("ORBextractor.nLevels").asInt().get(8);
    ini_th_fast = parameter("ORBextractor.iniThFAST").asInt().get(20);
    min_th_fast = parameter("ORBextractor.minThFAST").asInt().get(7);
    far_threshold = parameter("far_threshold").asDouble().get(4.0);
    imu_insert_kfs_when_lost = parameter("IMU.InsertKFsWhenLost").asBool().get(false);
    use_imu = parameter("use_imu").asBool().get(false);
    base_link_frame_id = parameter("base_link_frame_id").asString().get("base_link");

    transform.stamp.frame_id = "world";
    transform.child_frame_id = base_link_frame_id;

    auto vocabulary_path_o = parameter("vocabulary_path").asString().getStrict();
    auto slam_mode = parameter("mode").asString().getStrict({"Stereo", "RGBD", "Mono"});
    if (!vocabulary_path_o || !slam_mode) return POND_ERROR;
    
    vocabulary_path = *vocabulary_path_o;
    stereo_mode =  (slam_mode == "Stereo"); rgbd_mode = (slam_mode == "RGBD"); mono_mode = (slam_mode == "Mono");
    
    CameraInfo cam_info;
    if (!create_config(cam_info)) return POND_ERROR;

    GetFrameTransformRequest tf_request;
    if (!tfGetTransform(cam_info.stamp.frame_id, base_link_frame_id, 0, tf_request)) return POND_ERROR;
    cam_tf_start = tf_request.tf;

    transform_distributor = createDistributorTyped<FrameTransform>({"slam_transform"});
    keypoint_frame_distributor = createDistributorTyped<ImgFrameSPtr>({stereo_mode ? "mono_left_with_keypoints/image" : (rgbd_mode ? "color_with_keypoints/image" : "mono_with_keypoints/image")});

    new_frame.store(false);

    pond::ChannelsInfo channels_info;
    if (stereo_mode) channels_info.channels<ImgFrameSPtr, ImgFrameSPtr>("stereo_left/image", "stereo_right/image");
    if (rgbd_mode) channels_info.channels<ImgFrameSPtr, ImgFrameSPtr>("color/image", "depth/image");
    if (mono_mode) channels_info.channel<ImgFrameSPtr>("mono/image");

    image_receiver = createReceiver<ImgFrameSPtr>(channels_info, [this](ImgFrameSPtr** frames){

        std::lock_guard<std::mutex> lock(frame_mutex);

        new_frame.store(true);
        first_frame = *(frames[0]); if (!mono_mode) second_frame = *(frames[1]);
    });

    imu_data_points.reserve(100);
    imu_receiver = createReceiver<ImuData>({"imu"}, [this](ImuData* data){
    
        std::lock_guard<std::mutex> lock(imu_mutex);

        imu_data_points.push_back(ORB_SLAM3::IMU::Point(
            data->lin_acc[0], data->lin_acc[1], data->lin_acc[2],
            data->ang_vel[0], data->ang_vel[1], data->ang_vel[2],
            data->stamp.hw_time
        ));
    });

    ORB_SLAM3::System::eSensor mode;
    if (stereo_mode) mode = (use_imu ? ORB_SLAM3::System::IMU_STEREO : ORB_SLAM3::System::STEREO);
    if (rgbd_mode) mode = (use_imu ? ORB_SLAM3::System::IMU_RGBD : ORB_SLAM3::System::RGBD);
    if (mono_mode) mode = (use_imu ? ORB_SLAM3::System::IMU_MONOCULAR : ORB_SLAM3::System::MONOCULAR);

    slam = std::make_shared<ORB_SLAM3::System>(
        vocabulary_path,
        SETTINGS_PATH,
        mode,
        false
    );

    return POND_SUCCESS;
}

void OrbSlam3::onShutdownTF()
{
    slam->Shutdown();
    slam.reset();
   
    image_receiver.destroy();
    imu_receiver.destroy();

    transform_distributor.destroy();
    keypoint_frame_distributor.destroy();
}

bool OrbSlam3::create_config(CameraInfo& cam_info)
{
    GetFrameTransformRequest cam_request;

    if (stereo_mode)
    {
        CameraInfo cam_info_left, cam_info_right;
        if (!get_cam_info(cam_info_right, "stereo_right/info")) return false;
        if (!get_cam_info(cam_info_left, "stereo_left/info")) return false;

        if (!tfGetTransform(cam_info_right.stamp.frame_id, cam_info_left.stamp.frame_id, 0, cam_request))
            POND_LOG_RETURN_FALSE("Did not get camera optical frame transform");
    }
    
    std::string info_channel;
    if (stereo_mode) info_channel = "stereo_left/info";
    if (rgbd_mode) info_channel = "color/info"; 
    if (mono_mode) info_channel = "mono/info";
    if (!get_cam_info(cam_info, info_channel)) return false;

    std::ofstream file(SETTINGS_PATH);
    file << std::setprecision(16);
    file << "%YAML:1.0\n\n";
    file << "File.version: \"1.0\"\n\n";
    file << "Camera.type: \"Rectified\"\n\n";

    file << "Camera1.fx: " << cam_info.k(0, 0) << "\n";
    file << "Camera1.fy: " << cam_info.k(1, 1) << "\n";
    file << "Camera1.cx: " << cam_info.k(0, 2) << "\n";
    file << "Camera1.cy: " << cam_info.k(1, 2) << "\n\n";

    if (stereo_mode)
    {
        file << "Camera2.fx: " << cam_info.k(0, 0) << "\n";
        file << "Camera2.fy: " << cam_info.k(1, 1) << "\n";
        file << "Camera2.cx: " << cam_info.k(0, 2) << "\n";
        file << "Camera2.cy: " << cam_info.k(1, 2) << "\n\n";   
    }

    // Stereo baseline * focal length
    if (stereo_mode) file << "Camera.bf: " << (cam_info.k(0, 0) * cam_request.tf.translation().x()) << "\n\n";

    file << "Camera.width: " << cam_info.width << "\n";
    file << "Camera.height: " << cam_info.height << "\n";
    file << "Camera.fps: " << cam_info.fps << "\n\n";
    
    if (rgbd_mode)
    {
        file << "RGBD.DepthMapFactor: 1.0\n";
        file << "Camera.RGB: " << (cam_info.format == ImgFrame::Format::RGB8 ? 1 : 0) << "\n";
    }

    double base_line = (stereo_mode ? cam_request.tf.translation().x() : 0.1);
    file << "Stereo.ThDepth: " << (far_threshold / base_line)  << "\n";
    file << "Stereo.b: " << base_line << "\n\n";

    if (use_imu)
    {
        ImuInfo imu_info;
        if (!get_imu_info(imu_info)) return false;
        GetFrameTransformRequest tf_request;
        if (!tfGetTransform(imu_info.stamp.frame_id, cam_info.stamp.frame_id, 0, tf_request, true)) return false;

        Eigen::Matrix4d mat = tf_request.tf.matrix();
        
        file << "IMU.T_b_c1: !!opencv-matrix\n";
        file << "  rows: 4\n";
        file << "  cols: 4\n";
        file << "  dt: f\n";

        for (uint32_t i = 0; i < 4; i++)
        {
            for (uint32_t j = 0; j < 4; j++) {
                file << mat(i, j);

                if (i == 3 && j == 3) file << "]\n"; else file << ", ";
            }
            file << "\n  ";
        }

        file << "IMU.InsertKFsWhenLost: " << (imu_insert_kfs_when_lost ? 1 : 0) << "\n";

        file << "IMU.NoiseGyro: " << imu_info.ang_vel.noise << "\n";
        file << "IMU.NoiseAcc: " << imu_info.lin_acc.noise << "\n";
        file << "IMU.GyroWalk: " << imu_info.ang_vel.random_walk << "\n";
        file << "IMU.AccWalk: " << imu_info.lin_acc.random_walk << "\n";
        file << "IMU.Frequency: " << (double)imu_info.rate << "\n";
    }

    file << "ORBextractor.nFeatures: " << n_features << "\n";
    file << "ORBextractor.scaleFactor: " << scale_factor << "\n";
    file << "ORBextractor.nLevels: " << n_levels << "\n";
    file << "ORBextractor.iniThFAST: " << ini_th_fast << "\n";
    file << "ORBextractor.minThFAST: " << min_th_fast << "\n";

    file.close();

    return true;
}

void OrbSlam3::onFrame()
{
    ImgFrameSPtr first_frame_c, second_frame_c;
    std::vector<ORB_SLAM3::IMU::Point> imu_data_points_c;

    if (new_frame.load())
    {
        std::lock_guard<std::mutex> lock(frame_mutex);
        new_frame.store(false);

        first_frame_c = std::move(first_frame); if (!mono_mode) second_frame_c = std::move(second_frame);
    }
    else { pond::sleep(0.01); return; }

    {
        std::lock_guard<std::mutex> lock(imu_mutex);
        imu_data_points_c = std::move(imu_data_points);
        imu_data_points.clear();
    }

    cv::Mat no_keypoint_mat;
    Sophus::SE3f Tcw;

    if (stereo_mode)
    {
        if (first_frame_c->format != ImgFrame::Format::Mono8 || first_frame_c->format != ImgFrame::Format::Mono8)
            POND_LOG_RETURN("ERROR: left->format (%s) != Mono8 || right->format (%s) != Mono8", ImgFrame::formatToString(first_frame_c->format).c_str(), ImgFrame::formatToString(second_frame_c->format).c_str());

        cv::Mat left(first_frame_c->height, first_frame_c->width, CV_8UC1, first_frame_c->data);
        cv::Mat right(second_frame_c->height, second_frame_c->width, CV_8UC1, second_frame_c->data);

        transform.stamp.hw_time = (first_frame_c->stamp.hw_time + second_frame_c->stamp.hw_time) / 2.0;
        transform.stamp.time = (first_frame_c->stamp.time + second_frame_c->stamp.time) / 2.0;

        Tcw = slam->TrackStereo(left, right, transform.stamp.hw_time, imu_data_points_c);
        no_keypoint_mat = left;
    }
    if (rgbd_mode)
    {
        if (first_frame_c->format != ImgFrame::Format::RGB8 || first_frame_c->format != ImgFrame::Format::Depth16)
            POND_LOG_RETURN("ERROR: rgb->format (%s) != RGB8 || depth->format (%s) != Depth16", ImgFrame::formatToString(first_frame_c->format).c_str(), ImgFrame::formatToString(second_frame_c->format).c_str());

        cv::Mat rgb(first_frame_c->height, first_frame_c->width, CV_8UC3, first_frame_c->data);
        cv::Mat depth(second_frame_c->height, second_frame_c->width, CV_8UC2, second_frame_c->data);
        cv::Mat depth32;
        depth.convertTo(depth32, CV_32F, second_frame_c->depth_scale);

        transform.stamp.hw_time = (first_frame_c->stamp.hw_time + second_frame_c->stamp.hw_time) / 2.0;
        transform.stamp.time = (first_frame_c->stamp.time + second_frame_c->stamp.time) / 2.0;
        Tcw = slam->TrackRGBD(rgb, depth32, transform.stamp.hw_time, imu_data_points_c);
        no_keypoint_mat = rgb;
    }
    if (mono_mode)
    {
        if (first_frame_c->format != ImgFrame::Format::Mono8)
            POND_LOG_RETURN("ERROR: frame->format (%s) != Mono8", ImgFrame::formatToString(first_frame_c->format).c_str());

        cv::Mat mono(first_frame_c->height, first_frame_c->width, CV_8UC1, first_frame_c->data);

        transform.stamp.hw_time = first_frame_c->stamp.hw_time;
        transform.stamp.time = first_frame_c->stamp.time;
        Tcw = slam->TrackMonocular(mono, transform.stamp.hw_time, imu_data_points_c);
        no_keypoint_mat = mono;
    }

    GetFrameTransformRequest tf_request;
    if(!Tcw.matrix().isZero()) if (tfGetTransform(first_frame_c->stamp.frame_id, base_link_frame_id, first_frame_c->stamp.time, tf_request))
    {
        transform.tf = tf_request.tf * cam_tf_start.inverse() * Tcw.cast<double>();
        transform.stamp.time = first_frame_c->stamp.time;
        transform.stamp.hw_time = first_frame_c->stamp.hw_time;
        transform_distributor.distribute(&transform);
    }

    std::vector<cv::KeyPoint> keypoints = slam->GetTrackedKeyPointsUn();

    cv::Mat keypoint_mat;
    cv::drawKeypoints(
        no_keypoint_mat,
        keypoints,
        keypoint_mat,
        cv::Scalar(0, 255, 0)
    );

    ImgFrameSPtr keypoint_frame = std::make_shared<CVImgFrame>(keypoint_mat, rgbd_mode ? ImgFrame::Format::RGB8 : ImgFrame::Format::BGR8);
    keypoint_frame_distributor.distribute(&keypoint_frame);
}
