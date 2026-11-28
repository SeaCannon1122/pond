#include "pond/hpp/module_base.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <pond/pond.hpp>
#include <pond_data_types/command_types.hpp>
#include <pond_data_types/imu_types.hpp>
#include <pond_data_types/transform_types.hpp>
#include <pond_data_types/trajectory_types.hpp>
#include <sophus/so3.hpp>

class InputSynchronizer : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
private:

    pond::Callback twist_callback, pose_callback, stage_callback, imu_callback, trajectory_point_callback;
    std::mutex cmd_mutex, state_mutex;

    TwistCommand twist_cmd_rec;
    FrameTransform pose_cmd_rec;
    ImuData imu_data, imu_data_rec;
    TrajectoryPoint point, point_rec, cmd_point;
    
    struct
    {
        struct { double roll, pitch, height_min, height_max, height_start, x_vel, y_vel, yaw_vel; } cmd;
        struct { double roll, pitch; } operatable;
    } bounds;

    double timeout;
    std::string stage_name, base_link_name;

    bool external_trajectory_point; std::atomic<bool> invalid_orientation{false};
};

POND_MODULE_CPP_DECLARE(InputSynchronizer, "input_synchronizer", "gathers robot commands")

pond_result InputSynchronizer::onStartup(const std::vector<void*>& args)
{
    timeout =            parameter("timeout")           .asDouble()     .get(0.5);
    stage_name =         parameter("stage_name")        .asString()     .get("feed_external_inputs");
    base_link_name =     parameter("base_link_name")    .asString()     .get("base_link");

    bounds.cmd.roll =         parameter("bounds.cmd.roll")        .asDouble().get(0.8);
    bounds.cmd.pitch =        parameter("bounds.cmd.pitch")       .asDouble().get(0.4);
    bounds.cmd.height_min =   parameter("bounds.cmd.height_min")  .asDouble().get(0.04);
    bounds.cmd.height_max =   parameter("bounds.cmd.height_max")  .asDouble().get(0.3);
    bounds.cmd.height_start = parameter("bounds.cmd.height_start").asDouble().get(0.18);
    bounds.cmd.x_vel =        parameter("bounds.cmd.x_vel")       .asDouble().get(0.2);
    bounds.cmd.y_vel =        parameter("bounds.cmd.y_vel")       .asDouble().get(0.2);
    bounds.cmd.yaw_vel =      parameter("bounds.cmd.yaw_vel")     .asDouble().get(0.3);

    bounds.operatable.roll =  parameter("bounds.operatable.roll") .asDouble().get(1.2);
    bounds.operatable.pitch = parameter("bounds.operatable.pitch").asDouble().get(1.0);

    external_trajectory_point = parameter("external_trajectory_point").asBool().get(false);

    cmd_point.pose.translation().z() = bounds.cmd.height_start;

    twist_callback = createCallback<TwistCommand>({"cmd_vel"}, [this](TwistCommand* cmd) {
        std::lock_guard<std::mutex> lock(cmd_mutex);
        twist_cmd_rec = *cmd;
    });

    pose_callback = createCallback<FrameTransform>({"cmd_pose"}, [this](FrameTransform* pose) {
        std::lock_guard<std::mutex> lock(cmd_mutex);
        pose_cmd_rec = *pose;
    });

    imu_callback = createCallback<ImuData>({"imu/data"}, [this](ImuData* data_in) {
        std::lock_guard<std::mutex> lock(state_mutex);
        imu_data_rec = *data_in;
    });

    trajectory_point_callback = createCallback<TrajectoryPoint>({base_link_name+"_trajectory_point"}, [this](TrajectoryPoint* point_in) {
        std::lock_guard<std::mutex> lock(state_mutex);
        point_rec = *point_in;
    });

    stage_callback = createCallback<TrajectoryPoint*, ImuData*, TrajectoryPoint*>(
        {stage_name+"/cmd_trajectory_point", stage_name+"/imu_data", stage_name+"/robot_trajectory_point"},
        [this](TrajectoryPoint** cmd_trajectory_point_out, ImuData** imu_data_out, TrajectoryPoint** trajectory_point_out)
        {
            double time_now = pond::get_time(); 
            {
                std::lock_guard<std::mutex> lock(cmd_mutex);

                cmd_point.lin_vel.setZero(); cmd_point.ang_vel.setZero();

                if (twist_cmd_rec.stamp.time + timeout > time_now)
                {
                    cmd_point.lin_vel[0] = std::clamp(twist_cmd_rec.lin[0], -bounds.cmd.x_vel, bounds.cmd.x_vel);
                    cmd_point.lin_vel[1] = std::clamp(twist_cmd_rec.lin[1], -bounds.cmd.y_vel, bounds.cmd.y_vel);
                    cmd_point.lin_vel[2] = std::clamp(twist_cmd_rec.ang[2], -bounds.cmd.yaw_vel, bounds.cmd.yaw_vel);
                }

                if (pose_cmd_rec.stamp.time + timeout > time_now)
                {
                    Eigen::Vector3d rpy = pose_cmd_rec.tf.so3().matrix().eulerAngles(2, 1, 0).reverse();
                    cmd_point.rpy[0] = std::clamp(rpy[0], -bounds.cmd.roll, bounds.cmd.roll);
                    cmd_point.rpy[1] = std::clamp(rpy[1], -bounds.cmd.pitch, bounds.cmd.pitch);
                    cmd_point.use_rpy = true;
                    cmd_point.pose.translation().z() = std::clamp(pose_cmd_rec.tf.translation().z(), bounds.cmd.height_min, bounds.cmd.height_max);
                }
            }
            {
                std::lock_guard<std::mutex> lock(state_mutex);
                
                Eigen::Matrix3d orientation_mat = imu_data_rec.orientation.matrix();
                double roll  = std::atan2(orientation_mat(2, 1), orientation_mat(2, 2));
                double pitch = std::asin(std::clamp(-orientation_mat(2, 0), -1.0, 1.0));
                
                if (-bounds.operatable.roll > roll || bounds.operatable.roll < roll || -bounds.operatable.pitch > pitch || bounds.operatable.pitch < pitch) POND_LOG_RETURN("Not operatable at this orientation (roll: %f  pitch: %f)", roll, pitch);

                *cmd_trajectory_point_out = &cmd_point;

                if (!external_trajectory_point);
                else if (point_rec.stamp.time == 0) POND_LOG("No external trajectory point to feed in");
                else
                {
                    point = point_rec;
                    *trajectory_point_out = &point;
                }

                if (imu_data_rec.stamp.time == 0) POND_LOG("No IMU data to feed in");
                else {imu_data = imu_data_rec; *imu_data_out = &imu_data;}
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