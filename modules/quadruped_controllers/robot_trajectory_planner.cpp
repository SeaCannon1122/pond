#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>

#include <pond_data_types/trajectory_types.hpp>

class RobotTrajectoryPlanner : public pond::ModuleBaseTF
{
public:
    virtual pond_result onStartupTF(const std::vector<void*>& args) override;
    virtual void onShutdownTF() override;
private:
    pond::Callback stage_callback;

    double dt;
    uint32_t step_count;
    std::vector<TrajectoryPoint> traj;
};

POND_MODULE_CPP_DECLARE(RobotTrajectoryPlanner, "robot_trajectory_planner", "plans the target traj the robot body should follow")

pond_result RobotTrajectoryPlanner::onStartupTF(const std::vector<void*>& args)
{
    dt = parameter("dt").asDouble().get(0.03);
    step_count = parameter("step_count").asInt().get(12);
    traj.resize(step_count+1);

    stage_callback = createCallback<
        std::array<Eigen::Vector3d, 4>*, std::array<Eigen::Vector3d, 4>*,
        TrajectoryPoint*,                TrajectoryPoint*,                std::vector<TrajectoryPoint>*
    >({
        "plan_trajectory/joint_positions",      "plan_trajectory/joint_velocities",
        "plan_trajectory/cmd_trajectory_point", "plan_trajectory/robot_trajectory_point", "plan_trajectory/robot_target_trajectory"
    },[this](
        std::array<Eigen::Vector3d, 4>** joint_positions_in, std::array<Eigen::Vector3d, 4>** joint_velocities_in,
        TrajectoryPoint**                cmd_point_in,       TrajectoryPoint**                trajectory_point_in, std::vector<TrajectoryPoint>**   trajectory_out
    )
        {
            if (*joint_positions_in  == NULL) POND_LOG_RETURN("plan_trajectory/joint_positions is NULL");
            if (*joint_velocities_in == NULL) POND_LOG_RETURN("plan_trajectory/joint_velocities is NULL");
            if (*cmd_point_in        == NULL) POND_LOG_RETURN("plan_trajectory/cmd_trajectory_point is NULL");
            if (*trajectory_point_in == NULL) POND_LOG_RETURN("plan_trajectory/robot_trajectory_point is NULL");

            *trajectory_out = &traj;

            // optimal current state
            traj[0].pose.translation() = Eigen::Vector3d(0,0, cmd_point_in[0]->pose.translation().z());
            traj[0].rpy = Eigen::Vector3d(cmd_point_in[0]->rpy.x(), cmd_point_in[0]->rpy.y(), 0);

            traj[0].ang_vel = cmd_point_in[0]->ang_vel;
            traj[0].lin_vel = cmd_point_in[0]->lin_vel;
            
            // traget future states
            for (uint32_t i = 1; i < step_count+1; i++)
            {
                traj[i].stamp.time = dt*i;
                traj[i].use_rpy = true;

                traj[i].rpy                = traj[i-1].rpy                + dt * traj[i-1].ang_vel;
                traj[i].pose.translation() = traj[i-1].pose.translation() + dt * traj[i-1].lin_vel;
                
                traj[i].ang_vel = traj[i-1].ang_vel;
                traj[i].lin_vel = Sophus::SO3d::rotZ(traj[i].rpy.z()) * traj[0].lin_vel;
            }

            // actual current state
            Sophus::SO3d R_world = trajectory_point_in[0]->pose.so3();
            Eigen::Matrix3d R = R_world.matrix();

            double yaw = std::atan2(R(1, 0), R(0, 0));
            Sophus::SO3d yaw_inv = Sophus::SO3d::rotZ(-yaw);

            Sophus::SO3d R_aligned = yaw_inv * R_world;
            Eigen::Matrix3d Ra = R_aligned.matrix();

            double pitch = std::asin(std::clamp(-Ra(2, 0), -1.0, 1.0));
            double roll  = std::atan2(Ra(2, 1), Ra(2, 2));

            traj[0].stamp.time = 0;
            traj[0].use_rpy = true;
            traj[0].pose.translation() = Eigen::Vector3d(0, 0, trajectory_point_in[0]->pose.translation().z());
            traj[0].pose.so3() = R_aligned;
            traj[0].rpy = Eigen::Vector3d(roll, pitch, 0.0);

            traj[0].lin_vel = yaw_inv * trajectory_point_in[0]->lin_vel;
            traj[0].ang_vel = yaw_inv * trajectory_point_in[0]->ang_vel;
        }
    );

    return POND_SUCCESS;
}

void RobotTrajectoryPlanner::onShutdownTF()
{
    stage_callback.destroy();
}