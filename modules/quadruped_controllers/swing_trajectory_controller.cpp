#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>
#include <pond_data_types/command_types.hpp>

#include "swing_trajectory.hpp"

class SwingTrajectoryController : public pond::ModuleBaseTF
{
public:
    virtual pond_result onStartupTF(const std::vector<void*>& args) override;
    virtual void onShutdownTF() override;
private:
    std::array<Eigen::Vector3d, 4> foot_base_positions;

    double step_height;
    double max_step_length;
    double max_step_speed;

    double jerk_times_duration_to_the_3;
    double stance_amplitude;
    double T_0_in_swing_relative, T_1_in_swing_relative;

    std::array<SwingTrajectory, 4> trajectories;

    std::vector<TrajectoryPoint> trajectory_states;

    pond::Callback stage_callback;
};

POND_MODULE_CPP_DECLARE(SwingTrajectoryController, "swing_trajectory_controller", "calculating and evaluating the foot-swing-trajectories")

pond_result SwingTrajectoryController::onStartupTF(const std::vector<void*>& args)
{
    step_height         = parameter("step_height")      .asDouble().get(0.08);
    max_step_length     = parameter("max_step_length")  .asDouble().get(1.0);
    max_step_speed      = parameter("max_step_speed")   .asDouble().get(10.0);
    
    jerk_times_duration_to_the_3    = parameter("jerk_times_duration_to_the_3") .asDouble().get(18.75);
    stance_amplitude                = parameter("stance_amplitude")             .asDouble().get(0.02);
    T_0_in_swing_relative           = parameter("T_0_in_swing_relative")        .asDouble().get(0.25);
    T_1_in_swing_relative        = parameter("T_1_in_swing_relative")     .asDouble().get(0.75);

    auto foot_base_offset_vec        = parameter("foot_base_offset").asDoubleArray().get({0.015, 0.025, -0.181}, 3, 3);

    for (uint32_t i = 0; i < 4; i++)
    {
        foot_base_positions[i][0] = foot_base_offset_vec[0] * (i/2 ? -1.0 : 1.0);
        foot_base_positions[i][1] = foot_base_offset_vec[1] * (i%2 ? -1.0 : 1.0);
        foot_base_positions[i][2] = foot_base_offset_vec[2];
    }

    trajectory_states.resize(4);

    stage_callback = createCallback<
        std::vector<double>*, 
        std::vector<double>*, 
        TwistCommand*, 
        FrameTransform*,
        std::vector<TrajectoryPoint>*
    >(
        {
            "evaluate_swing_trajectories/gait_periods",
            "evaluate_swing_trajectories/partial_gait_durations",
            "evaluate_swing_trajectories/cmd_vel",
            "evaluate_swing_trajectories/cmd_pose",
            "evaluate_swing_trajectories/target_trajectory_states",
        },
        [this](
            std::vector<double>** periods_in,
            std::vector<double>** partial_durations_in,
            TwistCommand** twist_cmd_in,
            FrameTransform** pose_tf_in,
            std::vector<TrajectoryPoint>** target_trajectory_states_out
        )
        {
            if (*periods_in == NULL) POND_LOG_RETURN("evaluate_swing_trajectories/gait_periods is NULL");
            if (*partial_durations_in == NULL) POND_LOG_RETURN("evaluate_swing_trajectories/partial_gait_durations is NULL");
            if (*twist_cmd_in == NULL) POND_LOG_RETURN("evaluate_swing_trajectories/cmd_vel is NULL");
            if (*pose_tf_in == NULL) POND_LOG_RETURN("evaluate_swing_trajectories/cmd_pose is NULL");

            *target_trajectory_states_out = &trajectory_states;

            TwistCommand twist_cmd = **twist_cmd_in;
            double T_st = partial_durations_in[0][0][0], T_sw = partial_durations_in[0][0][1];
            double gait_duration = T_st + T_sw;
            
            if (T_sw > 0.0)
            {
                std::array<Eigen::Vector2d, 4> foot_velocities;
                {
                    for (uint32_t i = 0; i < 4; i++) foot_velocities[i] =
                    {
                        twist_cmd.lin[0] - twist_cmd.ang[2] * foot_base_positions[i][1], 
                        twist_cmd.lin[1] + twist_cmd.ang[2] * foot_base_positions[i][0]
                    };
                    
                    double max_velocity = 0.0;

                    // find vector of max velocity
                    for(size_t i = 0; i < 4; i++)
                    {
                        double vel = foot_velocities[i].norm();
                        
                        if (vel < 1e-5) foot_velocities[i] = Eigen::Vector2d::Zero();

                        else max_velocity = std::max(vel, max_velocity);
                    }

                    double MAX_VELOCITY = std::min(max_step_length / T_st, max_step_speed); 

                    // Check if the maximum of the velocities is bigger than the absolute maximum
                    if (max_velocity > MAX_VELOCITY) for (size_t i = 0; i < 4; i++) foot_velocities[i] *= MAX_VELOCITY / max_velocity;
                }

                for (uint32_t i = 0; i < 4; i++) trajectories[i].init(
                    T_st, T_sw, 
                    stance_amplitude,
                    T_0_in_swing_relative * T_sw, 
                    T_1_in_swing_relative * T_sw,
                    step_height, jerk_times_duration_to_the_3 / (gait_duration*gait_duration*gait_duration),
                    foot_velocities[i]
                );
            }

            double time_now = pond::get_time();

            for(size_t i = 0; i < 4; i++)
            {
                if (T_sw > 0.0) trajectories[i].get_state(periods_in[0][0][i], trajectory_states[i]);
                else
                {
                    trajectory_states[i].pose.translation().setZero();
                    trajectory_states[i].lin_vel.setZero();
                    trajectory_states[i].lin_acc.setZero();
                }

                trajectory_states[i].pose.translation() += foot_base_positions[i];
            }
        }
        
    );

    return POND_SUCCESS;
}

void SwingTrajectoryController::onShutdownTF()
{
    stage_callback.destroy();
}
