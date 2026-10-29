#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>
#include <pond_data_types/command_types.hpp>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include "pond/hpp/dds.hpp"
#include "pond_data_types/transform_types.hpp"
#include "trajectory.hpp"

class TrajectoryPlanner : public pond::ModuleBaseTF
{
public:
    virtual pond_result onStartupTF(const std::vector<void*>& args) override;
    virtual void onShutdownTF() override;
private:
    std::array<Eigen::Vector3d, 4> foot_base_positions;
    std::vector<FrameTransform> foot_positions_msg;
    pond::DistributorTyped<std::vector<FrameTransform>> foot_position_distributor;

    double step_height;
    double max_step_length;
    double max_step_speed;

    double jerk_times_duration_to_the_3;
    double stance_amplitude;
    double T_0_in_swing_relative, T_1_in_swing_relative;

    std::array<DynamicTrajectory, 4> trajectories;

    std::vector<TrajectoryState> trajectory_states;
    TwistCommand last_twist_cmd{{}, {0.1, 0, 0}, {0, 0, 0}};
    double last_T_sw = -1, last_T_st = -1;

    pond::Callback stage_callback;
};

POND_MODULE_CPP_DECLARE(TrajectoryPlanner, "trajectory_planner", "calculating and evaluating the feet trajectories")

pond_result TrajectoryPlanner::onStartupTF(const std::vector<void*>& args)
{
    step_height         = parameter("step_height")      .asDouble().get(0.08);
    max_step_length     = parameter("max_step_length")  .asDouble().get(1.0);
    max_step_speed      = parameter("max_step_speed")   .asDouble().get(10.0);
    
    jerk_times_duration_to_the_3    = parameter("jerk_times_duration_to_the_3") .asDouble().get(18.75);
    stance_amplitude                = parameter("stance_amplitude")             .asDouble().get(0.02);
    T_0_in_swing_relative           = parameter("T_0_in_swing_relative")        .asDouble().get(0.25);
    T_1_in_swing_relative        = parameter("T_1_in_swing_relative")     .asDouble().get(0.75);

    auto joint_names = parameter("joint_names").asStringArray().getStrict(12, 12);
    auto reference_link = parameter("reference_link").asString().getStrict();
    if (!joint_names || !reference_link) return POND_ERROR;

    auto foot_base_offset_vec        = parameter("foot_base_offset").asDoubleArray().get({0.015, 0.025, -0.181}, 3, 3);

    GetJointInfoRequest joint_requests[2];
    GetFrameTransformRequest tf_requests[2];
    
    if (!tfGetJointInfo(joint_names->at(0), joint_requests[0])) return POND_ERROR;
    if (!tfGetJointInfo(joint_names->at(1), joint_requests[1])) return POND_ERROR;
    
    if (!tfGetTransform(joint_requests[0].parent_link_name, *reference_link, 0, tf_requests[0])) return POND_ERROR;
    if (!tfGetTransform(joint_requests[1].parent_link_name, joint_requests[0].child_link_name, 0, tf_requests[1])) return POND_ERROR;
    
    Sophus::SE3d total = tf_requests[0].tf * joint_requests[0].tf;

    Eigen::Vector3d foot_base_pos_dims(
        foot_base_offset_vec[0] + total.translation().x(),
        foot_base_offset_vec[1] + total.translation().y() + (tf_requests[1].tf * joint_requests[1].tf).translation().norm(),
        foot_base_offset_vec[2]
    );

    for (uint32_t i = 0; i < 4; i++)
    {
        foot_base_positions[i] = foot_base_pos_dims;
        if (i/2) foot_base_positions[i][0] *= -1;
        if (i%2) foot_base_positions[i][1] *= -1;
    }

    trajectory_states.resize(4);
    foot_positions_msg.resize(4);
    for (uint32_t i = 0; i < 4; i++) foot_positions_msg[i].stamp.frame_id = *reference_link;
    
    foot_position_distributor = createDistributorTyped<std::vector<FrameTransform>>({"foot_target_poses"});

    stage_callback = createCallback<
        std::vector<double>*, 
        std::vector<double>*, 
        TwistCommand*, 
        FrameTransform*,
        std::vector<TrajectoryState>*
    >(
        {
            "evaluate_trajectories/gait_periods",
            "evaluate_trajectories/partial_gait_durations",
            "evaluate_trajectories/twist_command",
            "evaluate_trajectories/robot_pose",
            "evaluate_trajectories/target_trajectory_states",
        },
        [this](
            std::vector<double>** periods_in,
            std::vector<double>** partial_durations_in,
            TwistCommand** twist_cmd_in,
            FrameTransform** pose_tf_in,
            std::vector<TrajectoryState>** target_trajectory_states_out
        )
        {
            if (*periods_in == NULL) POND_LOG_RETURN("evaluate_trajectories/gait_periods is NULL");
            if (*partial_durations_in == NULL) POND_LOG_RETURN("evaluate_trajectories/partial_gait_durations is NULL");
            if (*twist_cmd_in == NULL) POND_LOG_RETURN("evaluate_trajectories/twist_command is NULL");
            if (*pose_tf_in == NULL) POND_LOG_RETURN("evaluate_trajectories/robot_pose is NULL");

            *target_trajectory_states_out = &trajectory_states;

            if (
                twist_cmd_in[0]->lin[0] != last_twist_cmd.lin[0] || 
                twist_cmd_in[0]->lin[1] != last_twist_cmd.lin[1] || 
                twist_cmd_in[0]->ang[2] != last_twist_cmd.ang[0] ||
                last_T_st != partial_durations_in[0][0][0] ||
                last_T_sw != partial_durations_in[0][0][1]
            )
            {
                last_twist_cmd = **twist_cmd_in;
                last_T_st = partial_durations_in[0][0][0];
                last_T_sw = partial_durations_in[0][0][1];
                double gait_duration = last_T_st + last_T_sw;

                std::array<Eigen::Vector2d, 4> foot_velocities;
                {
                    for (uint32_t i = 0; i < 4; i++) foot_velocities[i] =
                    {
                        last_twist_cmd.lin[0] - last_twist_cmd.ang[2] * foot_base_positions[i][1], 
                        last_twist_cmd.lin[1] + last_twist_cmd.ang[2] * foot_base_positions[i][0]
                    };
                    
                    double epsilon = 1e-5, max_velocity = 0.0;

                    // find vector of max velocity
                    for(size_t i = 0; i < 4; i++)
                    {
                        foot_positions_msg[i].tf.so3() = Sophus::SO3d::rotZ(atan2(foot_velocities[i][1], foot_velocities[i][0]));

                        double vel = foot_velocities[i].norm();
                        
                        if (vel < epsilon) foot_velocities[i] = Eigen::Vector2d::Zero();

                        else max_velocity = std::max(vel, max_velocity);
                    }

                    double MAX_VELOCITY = std::min(max_step_length / last_T_st, max_step_speed); 

                    // Check if the maximum of the velocities is bigger than the absolute maximum
                    if (max_velocity > MAX_VELOCITY) for (size_t i = 0; i < 4; i++) foot_velocities[i] *= MAX_VELOCITY / max_velocity;
                }

                for (uint32_t i = 0; i < 4; i++) trajectories[i].init(
                    last_T_st, last_T_sw, 
                    stance_amplitude,
                    T_0_in_swing_relative * last_T_sw, 
                    T_1_in_swing_relative * last_T_sw,
                    step_height, jerk_times_duration_to_the_3 / (gait_duration*gait_duration*gait_duration),
                    foot_velocities[i]
                );
            }

            for(size_t i = 0; i < 4; i++)
            {
                trajectories[i].get_state(periods_in[0][0][i], trajectory_states[i]);

                trajectory_states[i].pos += foot_base_positions[i];

                foot_positions_msg[i].tf.translation() = trajectory_states[i].pos;
            }

            foot_position_distributor.distribute(&foot_positions_msg);
        }
        
    );

    return POND_SUCCESS;
}

void TrajectoryPlanner::onShutdownTF()
{
    stage_callback.destroy();
    foot_position_distributor.destroy();
}
