#define POND_MODULE_CPP_MAKE_IMPLEMENTATION
#include <pond/module_base_tf.hpp>
#include <pond/data_types/cv_img_frame.hpp>
#include <pond/data_types/imu_types.hpp>

#include "System.h"

class OrbSlam3 : public pond::ModuleBaseTF
{
public:
    virtual pond_result onStartupTF(const std::vector<void*>& args) override;
    virtual void onShutdownTF() override;
    virtual void onFrame() override;
private:

    bool get_cam_info(CameraInfo& cam_info, const std::string& topic);
    bool get_imu_info(ImuInfo& info);
    bool create_config();

    std::shared_ptr<ORB_SLAM3::System> slam;
    bool slam_setup = false;
    std::string vocabulary_path;

    ORB_SLAM3::System::eSensor mode;
    uint32_t n_features;
    double scale_factor;
    uint32_t n_levels;
    uint32_t ini_th_fast;
    uint32_t min_th_fast;
    double far_threshold;
    bool imu_insert_kfs_when_lost;

    std::mutex frame_mutex;
    std::mutex imu_mutex;
    std::vector<ORB_SLAM3::IMU::Point> imu_data_points;
    std::atomic<bool> new_frame;

    bool use_imu;

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
    POND_MODULE(OrbSlam3),
)

pond_result OrbSlam3::onStartupTF(const std::vector<void*>& args)
{
    n_features = parameter("ORBextractor.nFeatures").asInt().get(1250);
    scale_factor = parameter("ORBextractor.scaleFactor").asDouble().get(1.2);
    n_levels = parameter("ORBextractor.nLevels").asInt().get(8);
    ini_th_fast = parameter("ORBextractor.iniThFAST").asInt().get(20);
    min_th_fast = parameter("ORBextractor.minThFAST").asInt().get(7);
    far_threshold = parameter("far_threshold").asDouble().get(4.0);
    imu_insert_kfs_when_lost = parameter("IMU.InsertKFsWhenLost").asBool().get(false);

    auto vocabulary_path_o = parameter("vocabulary_path").asString().getStrict();
    auto slam_mode = parameter("mode").asString().getStrict({"Stereo", "RGBD", "Mono"});
    if (!vocabulary_path_o || !slam_mode) return POND_ERROR;
    
    vocabulary_path = *vocabulary_path_o;

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

void OrbSlam3::onShutdownTF()
{
    if (slam_setup)
    {
        slam->Shutdown();
        slam.reset();
    }
    
    if (stereo.use) stereo.receiver.destroy();
    if (rgbd.use) rgbd.receiver.destroy();
    if (mono.use) mono.receiver.destroy();

    imu_receiver.destroy();

    transform_distributor.destroy();
    keypoint_frame_distributor.destroy();
}

#define SETTINGS_PATH "~/.cache/orbslam_config.yaml"

bool OrbSlam3::get_cam_info(CameraInfo& info, const std::string& topic)
{
    pond::Receiver<CameraInfo> info_receiver = createReceiver<CameraInfo>({topic}, [&](CameraInfo* info_msg) {if (info.stamp.time == 0) info = *info_msg;});
    for (int i = 0; i < 100 && info.stamp.time == 0; i++) this_thread::sleep_for(std::chrono::milliseconds(10));
    info_receiver.destroy();
    if (info.stamp.time == 0) {POND_LOG("Did not receive camera info on topic '%s'", topic.c_str()); return false;}
    return true;
}

bool OrbSlam3::get_imu_info(ImuInfo& info)
{
    pond::Receiver<ImuInfo> info_receiver = createReceiver<ImuInfo>({"imu/info"}, [&](ImuInfo* info_msg) {if (info.stamp.time == 0) info = *info_msg;});
    for (int i = 0; i < 100 && info.stamp.time == 0; i++) this_thread::sleep_for(std::chrono::milliseconds(10));
    info_receiver.destroy();
    if (info.stamp.time == 0) {POND_LOG("Did not receive imu info on topic 'imu/info'"); return false;}
    return true;
}

bool OrbSlam3::create_config()
{
    CameraInfo cam_info;
    
    GetFrameTransformRequest cam_request;
    if (stereo.use)
    {
        if (!get_cam_info(cam_info, "stereo_right/info")) return false;
        cam_request.source_frame = cam_info.stamp.frame_id;
        if (!get_cam_info(cam_info, "stereo_left/info")) return false;
        cam_request.target_frame = cam_info.stamp.frame_id;
        cam_request.time_point = 0;

        pond::Distributor<GetFrameTransformRequest> request_distributor = createDistributor<GetFrameTransformRequest>({"get_robot_transform"});
        request_distributor.distribute(cam_request);
        request_distributor.destroy();

        if (cam_request.fufilled == false) {POND_LOG("Did not get camera optical frame transform"); return false;}
    }
    
    std::string info_topic;
    if (stereo.use) info_topic = "stereo_left/info";
    if (rgbd.use) info_topic = "color/info"; 
    if (mono.use) info_topic = "mono/info";
    if (!get_cam_info(cam_info, info_topic)) return false;

    std::ofstream file(SETTINGS_PATH);
    file << "%YAML:1.0\n\n";
    file << "File.version: \"1.0\"\n\n";
    file << "Camera.type: \"Rectified\"\n\n";

    file << "Camera1.fx: " + std::to_string(cam_info.k(0, 0)) + "\n";
    file << "Camera1.fy: " + std::to_string(cam_info.k(1, 1)) + "\n";
    file << "Camera1.cx: " + std::to_string(cam_info.k(0, 2)) + "\n";
    file << "Camera1.cy: " + std::to_string(cam_info.k(1, 2)) + "\n\n";

    if (stereo.use)
    {
        file << "Camera2.fx: " + std::to_string(cam_info.k(0, 0)) + "\n";
        file << "Camera2.fy: " + std::to_string(cam_info.k(1, 1)) + "\n";
        file << "Camera2.cx: " + std::to_string(cam_info.k(0, 2)) + "\n";
        file << "Camera2.cy: " + std::to_string(cam_info.k(1, 2)) + "\n\n";   
    }

    // Stereo baseline * focal length
    if (stereo.use) file << "Camera.bf: " + std::to_string(cam_info.k(0, 0) * cam_request.tf.translation().x()) + "\n\n";

    file << "Camera.width: " + std::to_string(cam_info.width) + "\n";
    file << "Camera.height: " + std::to_string(cam_info.height) + "\n";
    file << "Camera.fps: " + std::to_string(cam_info.fps) + "\n\n";
    
    if (rgbd.use)
    {
        file << "RGBD.DepthMapFactor: 1.0\n";
        file << "Camera.RGB: " + std::to_string(cam_info.format == ImgFrame::Format::RGB8 ? 1 : 0) + "\n";
    }

    double base_line = (stereo.use ? cam_request.tf.translation().x() : 0.1);
    file << "Stereo.ThDepth: " + std::to_string(far_threshold / base_line) + "\n";
    file << "Stereo.b: " + std::to_string(base_line) + "\n\n";

    // if (use_imu)
    // {
    //     ImuInfo imu_info;
    //     if (!get_imu_info(imu_info)) return false;
    //     GetFrameTransformRequest tf_request;
    //     if (!tfGetTransform(imu_info.stamp.frame_id, cam_info.stamp.frame_id, 0, tf_request, true)) return false;

    //     file << "IMU.T_b_c1: !!opencv-matrix\n";
    //     file << "  rows: 4\n";
    //     file << "  cols: 4\n";
    //     file << "  dt: f\n";
    //     file << "  data: [0.999903, -0.0138036, -0.00208099, -0.0202141,\n";
    //     file << "  0.0137985, 0.999902, -0.00243498, 0.00505961,\n";
    //     file << "  0.0021144, 0.00240603, 0.999995, 0.0114047,\n";
    //     file << "  0.0, 0.0, 0.0, 1.0]\n";

    //     file << "IMU.InsertKFsWhenLost: " + std::to_string(imu_insert_kfs_when_lost ? 1 : 0) + "\n";

    //     # IMU noise (Use those from VINS-mono)
    //     file << "IMU.NoiseGyro: 1e-2 # 3 # 2.44e-4 #1e-3 # rad/s^0.5";
    //     file << "IMU.NoiseAcc: 1e-1 #2 # 1.47e-3 #1e-2 # m/s^1.5";
    //     file << "IMU.GyroWalk: 1e-6 # rad/s^1.5";
    //     file << "IMU.AccWalk: 1e-4 # m/s^2.5";
    //     file << "IMU.Frequency: " + std::to_string((double)imu_info.rate) + "\n";
    // }

    file << "ORBextractor.nFeatures: " + std::to_string(n_features) + "\n";
    file << "ORBextractor.scaleFactor: " + std::to_string(scale_factor) + "\n";
    file << "ORBextractor.nLevels: " + std::to_string(n_levels) + "\n";
    file << "ORBextractor.iniThFAST: " + std::to_string(ini_th_fast) + "\n";
    file << "ORBextractor.minThFAST: " + std::to_string(min_th_fast) + "\n";

    return true;
}

void OrbSlam3::onFrame()
{
    if (!slam_setup)
    {
        slam = std::make_shared<ORB_SLAM3::System>(
            vocabulary_path,
            SETTINGS_PATH,
            ORB_SLAM3::System::STEREO,
            false
        );
    }

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
