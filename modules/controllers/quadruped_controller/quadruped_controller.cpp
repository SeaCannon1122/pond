#include <Eigen/src/Core/Matrix.h>
#include <array>
#include <cmath>
#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>

#include <pond_data_types/command_types.hpp>
#include <pond_data_types/motor_types.hpp>

#include "pond/pond.h"
#include "trajectory.hpp"

class QuadrupedController : public pond::ModuleBaseTF
{
public:
    virtual pond_result onStartupTF(const std::vector<void*>& args) override;
    virtual void onShutdownTF() override;
    void update_controller(MotorInterface** ifs);

private:

    std::array<Eigen::Vector3d, 4> generateBasePosition(bool extra) const;
    std::array<Eigen::Vector2d, 4> calculateFootVelocities(const TwistCommand& twist) const;

    double step_height_;
    double max_step_length_;
    double stand_height_;
    double max_step_speed_;
    double robot_width_;
    double robot_length_;
    double limb_length_hip_to_shoulder_;
    double limb_length_thigh_;
    double limb_length_shin_;
    double jerk_;
    double stance_amplitude_;
    double T_st_, T_sw_, T_0_, T_1_;

    pond::Receiver twist_receiver, update_receiver, locomotion_receiver;
    pond::DistributorTyped<std::vector<double>> gait_distributor; 
    pond::DistributorTyped<std::vector<FrameTransform>> foot_position_distributor; 
    std::vector<FrameTransform> foot_positions_msg;

    std::mutex twist_mutex;
    TwistCommand twist_cmd, twist;
    double timeout;

    std::array<DynamicTrajectory, 4> trajectories;
    std::atomic<bool> locomotion_activated{true};

    std::vector<JointState> joint_states;
};

POND_MODULE_CPP_DECLARE(QuadrupedController, "quadruped_controller", "robodog controller")

pond_result QuadrupedController::onStartupTF(const std::vector<void*>& args)
{
    timeout = parameter("timeout").asDouble().get(0.5);

    robot_width_                    = parameter("robot_width")                  .asDouble().get(0.07);
    robot_length_                   = parameter("robot_length")                 .asDouble().get(0.26);
    
    limb_length_hip_to_shoulder_    = parameter("limb_length_hip_to_shoulder")  .asDouble().get(0.065);
    limb_length_thigh_              = parameter("limb_length_thigh")            .asDouble().get(0.105);
    limb_length_shin_               = parameter("limb_length_shin")             .asDouble().get(0.115);

    T_0_                            = parameter("T_0")                          .asDouble().get(0.1);
    T_1_                            = parameter("T_1")                          .asDouble().get(0.3);
    T_st_                           = parameter("T_st")                         .asDouble().get(0.4);
    T_sw_                           = parameter("T_sw")                         .asDouble().get(0.4);
    step_height_                    = parameter("step_height")                  .asDouble().get(0.08);
    max_step_length_                = parameter("max_step_length")              .asDouble().get(1.0);
    stand_height_                   = parameter("stand_height")                 .asDouble().get(0.18);
    max_step_speed_                 = parameter("max_step_speed")               .asDouble().get(10.0);
    jerk_                           = parameter("jerk")                         .asDouble().get(800.0);
    stance_amplitude_               = parameter("stance_amplitude")             .asDouble().get(0.02);

    auto joint_names = parameter("joint_names").asStringArray().getStrict(12, 12);
    auto motor_names = parameter("motor_names").asStringArray().getStrict(12, 12);
    if (!joint_names || !motor_names) return POND_ERROR;

    gait_distributor = createDistributorTyped<std::vector<double>>({"gaits"});
    foot_position_distributor = createDistributorTyped<std::vector<FrameTransform>>({"foot_target_positions"});
    foot_positions_msg.resize(4);
    foot_positions_msg[0].stamp.frame_id = "base_link";

    std::array<Eigen::Vector2d, 4> foot_velocities = calculateFootVelocities(TwistCommand{.lin = {0.01, 0.0, 0.0}});

    for (uint32_t i = 0; i < 4; i++) trajectories[i].init(T_st_, T_sw_, stance_amplitude_, T_0_, T_1_, step_height_, jerk_, foot_velocities[i]);

    joint_states.resize(12);
    pond::ChannelsInfo channels_info;

    for (uint32_t i = 0; i < 12; i++)
    {
        joint_states[i].joint_name = joint_names->at(i);
        channels_info.channel<MotorInterface>(motor_names->at(i) + "/update");
    }

    update_receiver = createReceiver<MotorInterface>(channels_info, std::bind(&QuadrupedController::update_controller, this, std::placeholders::_1));

    locomotion_receiver = createReceiver<bool>({"enable_locomotion"}, [this](bool* status) {locomotion_activated.store(status);});    

    twist_receiver = createReceiver<TwistCommand>({"cmd_vel"}, [this](TwistCommand* new_twist){
        
        if (new_twist->stamp.time + timeout < pond::get_time()) return;

        std::lock_guard<std::mutex> lock(twist_mutex);
        twist_cmd = *new_twist;
    });

    return POND_SUCCESS;
}

void QuadrupedController::onShutdownTF()
{
    update_receiver.destroy();
    twist_receiver.destroy();
    locomotion_receiver.destroy();
    gait_distributor.destroy();
    foot_position_distributor.destroy();
}

std::array<Eigen::Vector3d, 4> QuadrupedController::generateBasePosition(bool extra) const
{
    std::array<Eigen::Vector3d, 4> foot_targets;

    Eigen::Vector3d dims(
        robot_length_ / 2.0                                + (extra ? 0.015 : 0.0),
        robot_width_ / 2.0 + limb_length_hip_to_shoulder_  + (extra ? 0.025 : 0.0),
        -stand_height_
    );

    foot_targets[0] = { dims[0],  dims[1], dims[2]};
    foot_targets[1] = { dims[0], -dims[1], dims[2]};
    foot_targets[2] = {-dims[0],  dims[1], dims[2]};
    foot_targets[3] = {-dims[0], -dims[1], dims[2]};

    return foot_targets;
}

std::array<Eigen::Vector2d, 4> QuadrupedController::calculateFootVelocities(const TwistCommand& twist) const
{
  std::array<Eigen::Vector2d, 4> velocities;
  std::array<Eigen::Vector3d, 4> foot_positions = generateBasePosition(false);

  for(size_t i = 0; i < 4; i++) velocities[i] =
  {
    twist.lin[0] - twist.ang[2] * foot_positions[i][1], 
    twist.lin[1] + twist.ang[2] * foot_positions[i][0]
  };
  
  double epsilon = 1e-5, max_velocity = 0.0;

  // find vector of max velocity
  for(size_t i = 0; i < 4; i++)
  {
    double vel = velocities[i].norm();
    
    if (vel < epsilon) velocities[i] = Eigen::Vector2d::Zero();

    else max_velocity = std::max(vel, max_velocity);
  }

  // Check if the maximum of the velocities is bigger than the absolute maximum
  if (max_velocity > max_step_speed_) for (size_t i = 0; i < 4; i++) velocities[i] *= max_step_speed_ / max_velocity;

  return velocities;
}

bool twist_is_usable(TwistCommand& cmd)
{
    return (std::abs(cmd.lin[0]) > 0.001 | std::abs(cmd.lin[1]) > 0.001 | std::abs(cmd.ang[2]) > 0.001);
}

inline bool law_of_cosines(double l1, double l2, double b, double* theta)
{
    double acos_theta = (l1*l1 + l2*l2 - b*b)/(2*l1*l2);
    if (acos_theta > 1.0 || acos_theta < -1.0) return false;

    *theta = acos(acos_theta);

    return true;
}

void QuadrupedController::update_controller(MotorInterface** ifs)
{
    double time_now = pond::get_time();

    for (uint32_t i = 0; i < joint_states.size(); i++)
    {
        if (ifs[i]->feedback.pos) joint_states[i].angle = *ifs[i]->feedback.pos;
        joint_states[i].time = ifs[i]->feedback.time;
        joint_states[i].hw_time = ifs[i]->feedback.hw_time;
    }

    tfSetJointStates(joint_states);

    if(!locomotion_activated.load()) return;

    twist_mutex.lock();
    TwistCommand twist_cmd_copy = twist_cmd;
    twist_mutex.unlock();

    std::array<double, 4> relative_phase_offsets = {0.0, 0.5, 0.5, 0.0};
    std::vector<double> phase_array(4);
    double gait_period = T_st_ + T_sw_;

    bool twist_usable = twist_is_usable(twist_cmd_copy);

    if (twist_usable) for (size_t i = 0; i < 4; i++)
        phase_array[i] = std::fmod(time_now + relative_phase_offsets[i] * gait_period, gait_period);
    else phase_array = {T_st_ / 2.0, T_st_ / 2.0, T_st_ / 2.0, T_st_ / 2.0};
    
    gait_distributor.distribute(&phase_array);

    // Generate base positions and foot velocity vectors
    std::array<Eigen::Vector2d, 4> foot_velocities = calculateFootVelocities(twist_usable ? twist_cmd_copy : TwistCommand{.lin = {0.01, 0.0, 0.0}});

    if (twist_cmd_copy.lin[0] != twist.lin[0] || twist_cmd_copy.lin[1] != twist.lin[1] || twist_cmd_copy.ang[2] != twist.ang[0])
    {
        for (uint32_t i = 0; i < 4; i++) trajectories[i].init(T_st_, T_sw_, stance_amplitude_, T_0_, T_1_, step_height_, jerk_, foot_velocities[i]);

        twist = twist_cmd_copy;
    }

    std::array<Eigen::Vector3d, 4> base_positions = generateBasePosition(false);

    std::array<Eigen::Vector3d, 4> positions, velocities, accelerations;

    for(size_t i = 0; i < 4; i++)
    {
        trajectories[i].get_state(phase_array[i], positions[i], velocities[i], accelerations[i]);

        positions[i] += base_positions[i];

        foot_positions_msg[i].tf = Sophus::SE3d(Sophus::SO3d::rotZ(atan2(foot_velocities[i][1], foot_velocities[i][0])), positions[i] + Eigen::Vector3d(0.0, i%2 ? -0.025 : 0.025, 0.0));
        foot_positions_msg[i].stamp.time = time_now;
        foot_positions_msg[i].stamp.hw_time = time_now;

        double x = positions[i][0] - (i < 2 ? robot_length_ / 2 : -robot_length_ / 2);
        double y = positions[i][1] - (i % 2 == 0 ? robot_width_ / 2 : -robot_width_ / 2);
        double z = positions[i][2];

        if (i % 2 != 0) y = -y;
        if (z > 0) x*= -1;

        double knee_angle, shoulder_angle, hip_angle;

        double shoulder_to_foot_sq = z * z + y * y - limb_length_hip_to_shoulder_ * limb_length_hip_to_shoulder_;
        if (shoulder_to_foot_sq < 0) continue;
        double shoulder_to_foot = std::sqrt(shoulder_to_foot_sq);

        hip_angle = std::atan2(y, std::abs(z)) + std::atan2(shoulder_to_foot, limb_length_hip_to_shoulder_);

        double distance = std::sqrt(x * x + shoulder_to_foot * shoulder_to_foot);

        if (!law_of_cosines(limb_length_thigh_, limb_length_shin_, distance, &knee_angle)) continue;
        if (!law_of_cosines(limb_length_thigh_, distance, limb_length_shin_, &shoulder_angle)) continue;

        shoulder_angle -= std::atan2(x, shoulder_to_foot);
        if(z>0) shoulder_angle += M_PI;

        // Adjust angle from triangle perspective to robot perspective
        hip_angle -= M_PI_2;
        if (z>0) hip_angle *= -1;
        knee_angle -= M_PI;

        if (i%2) hip_angle *= -1;

        ifs[3*i+0]->command.pos = hip_angle;
        ifs[3*i+1]->command.pos = shoulder_angle;
        ifs[3*i+2]->command.pos = knee_angle;
    }

    foot_position_distributor.distribute(&foot_positions_msg);
}
