#include <pond/pond.hpp>
#include <pond_data_types/command_types.hpp>
#include <pond_data_types/imu_types.hpp>
#include <pond_data_types/transform_types.hpp>
#include <pond_data_types/trajectory_types.hpp>

class InputSynchronizer : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
private:

    pond::Callback twist_callback, pose_callback, stage_callback, imu_callback, trajectory_point_callback;
    std::mutex cmd_mutex, state_mutex;

    TwistCommand twist_cmd, twist_cmd_rec;
    FrameTransform pose_cmd, pose_cmd_rec;
    ImuData imu_data, imu_data_rec;
    ImuInfo imu_info, imu_info_rec;
    TrajectoryPoint point, point_rec;
    
    std::vector<double> height_bounds; // z_min z_max z_default
    std::vector<double> orientation_bounds; // roll pitch yaw
    std::vector<double> velocity_bounds; // linx liny angz
    double timeout;
    std::string stage_name, base_link_name;

    bool external_trajectory_point;
};

POND_MODULE_CPP_DECLARE(InputSynchronizer, "input_synchronizer", "gathers robot commands")

pond_result InputSynchronizer::onStartup(const std::vector<void*>& args)
{
    timeout =            parameter("timeout")           .asDouble()     .get(0.5);
    stage_name =         parameter("stage_name")        .asString()     .get("feed_external_inputs");
    base_link_name =     parameter("base_link_name")    .asString()     .get("base_link");

    velocity_bounds =    parameter("bounds.velocity")   .asDoubleArray().get({0.2, 0.2, 0.3}, 3, 3);
    height_bounds =      parameter("bounds.height")     .asDoubleArray().get({0.04, 0.3, 0.18}, 3, 3);
    orientation_bounds = parameter("bounds.orientation").asDoubleArray().get({0.8, 0.6, 0.3}, 3, 3);
    
    external_trajectory_point = parameter("external_trajectory_point").asBool().get(false);

    pose_cmd.tf.translation().z() = height_bounds[2];

    twist_callback = createCallback<TwistCommand>({"cmd_vel"}, [this](TwistCommand* cmd) {
        std::lock_guard<std::mutex> lock(cmd_mutex);
        twist_cmd_rec = *cmd;
    });

    pose_callback = createCallback<FrameTransform>({"cmd_pose"}, [this](FrameTransform* pose) {
        std::lock_guard<std::mutex> lock(cmd_mutex);
        pose_cmd_rec = *pose;
    });

    imu_callback = createCallback<ImuData, ImuInfo>({"imu/data", "imu/info"}, [this](ImuData* data_in, ImuInfo* info_in) {
        std::lock_guard<std::mutex> lock(state_mutex);
        imu_data_rec = *data_in;
        imu_info_rec = *info_in;
    });

    trajectory_point_callback = createCallback<TrajectoryPoint>({base_link_name+"_trajectory_point"}, [this](TrajectoryPoint* point_in) {
        std::lock_guard<std::mutex> lock(state_mutex);
        point_rec = *point_in;
    });

    stage_callback = createCallback<TwistCommand*, FrameTransform*, ImuData*, ImuInfo*, TrajectoryPoint*>(
        {stage_name+"/cmd_vel", stage_name+"/cmd_pose", stage_name+"/imu_data", stage_name+"/imu_info", stage_name+"/robot_trajectory_point"},
        [this](TwistCommand** twist_cmd_out, FrameTransform** pose_cmd_out, ImuData** imu_data_out, ImuInfo** imu_info_out, TrajectoryPoint** trajectory_point_out)
        {
            double time_now = pond::get_time(); 
            {
                std::lock_guard<std::mutex> lock(cmd_mutex);

                twist_cmd = TwistCommand(); twist_cmd.stamp.time = time_now;

                if (twist_cmd_rec.stamp.time + timeout > time_now)
                {
                    twist_cmd.lin[0] = std::clamp(twist_cmd_rec.lin[0], -velocity_bounds[0], velocity_bounds[0]);
                    twist_cmd.lin[1] = std::clamp(twist_cmd_rec.lin[1], -velocity_bounds[1], velocity_bounds[1]);
                    twist_cmd.ang[2] = std::clamp(twist_cmd_rec.ang[2], -velocity_bounds[2], velocity_bounds[2]);
                }

                pose_cmd.stamp.time = time_now;

                if (pose_cmd_rec.stamp.time + timeout > time_now)
                {
                    Eigen::Vector3d rpy = pose_cmd_rec.tf.so3().matrix().eulerAngles(2, 1, 0).reverse();

                    pose_cmd.tf = Sophus::SE3d(Eigen::Quaterniond(
                        Eigen::AngleAxisd(std::clamp(rpy[2], -orientation_bounds[2], orientation_bounds[2]), Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(std::clamp(rpy[1], -orientation_bounds[1], orientation_bounds[1]), Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(std::clamp(rpy[0], -orientation_bounds[0], orientation_bounds[0]), Eigen::Vector3d::UnitX())
                    ), Eigen::Vector3d(0, 0, std::clamp(pose_cmd_rec.tf.translation().z(), height_bounds[0], height_bounds[1])));
                }
            }
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                
                if (!external_trajectory_point);
                else if (point_rec.stamp.time == 0) POND_LOG("No external trajectory point to feed in");
                else
                {
                    point = point_rec;
                    *trajectory_point_out = &point;
                }

                if (imu_data_rec.stamp.time == 0 || imu_info_rec.stamp.time == 0) POND_LOG("No IMU data to feed in");
                else
                {
                    imu_data = imu_data_rec;
                    imu_info = imu_info_rec;

                    *twist_cmd_out = &twist_cmd;   *pose_cmd_out = &pose_cmd;
                    *imu_data_out = &imu_data;     *imu_info_out = &imu_info;
                }
            }
        }
    );

    return POND_SUCCESS;
}

void InputSynchronizer::onShutdown()
{
    stage_callback.destroy();
    twist_callback.destroy();
    pose_callback.destroy();
    imu_callback.destroy();
    trajectory_point_callback.destroy();
}