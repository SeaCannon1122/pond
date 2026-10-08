#include <cmath>
#include <functional>
#include <pond/pond.hpp>
#include <pond/hpp/module_base_tf.hpp>

#include <pond_data_types/command_types.hpp>
#include <pond_data_types/motor_types.hpp>
#include <vector>
#include "pond/hpp/dds.hpp"
#include "pond_data_types/robot_state_types.hpp"
#include "pond_data_types/transform_types.hpp"
#include "trajectory.hpp"

class Gait
{
public:
    Gait(std::array<double, 4> duty_cycle, std::array<double, 4> phase_offsets, double gait_period)
    : duty_cycle_(duty_cycle), phase_offsets_(phase_offsets), gait_period_(gait_period) {}

    double get_phase(std::size_t leg_index, double time_now) const {
        // Normalize time to [0, 1)
        double t_norm = std::fmod(time_now, gait_period_) / gait_period_;
        // Apply per-leg phase offset
        double leg_phase = std::fmod(t_norm - phase_offsets_[leg_index] + 1.0, 1.0);

        double duty = duty_cycle_[leg_index];

        // Swing phase → scale to [0.0, 0.5)
        if (leg_phase < duty) return (leg_phase / duty) * 0.5 + 0.5;
            
        // Stance phase → scale to [0.5, 1.0)
        else return ((leg_phase - duty) / (1.0 - duty)) * 0.5;
    }

private:
    std::array<double, 4> duty_cycle_;
    std::array<double, 4> phase_offsets_;
    double gait_period_;
};

Gait make_trot(double period) {
    return Gait({0.5, 0.5, 0.5, 0.5}, {0.0, 0.5, 0.5, 0.0}, period);
}

Gait make_walk(double period) {
    return Gait({0.25, 0.25, 0.25, 0.25}, {0.0, 0.25, 0.5, 0.75}, period);
}

Gait make_amble(double period) {
    return Gait({0.5, 0.5, 0.5, 0.5}, {0.0, 0.5, 0.0, 0.5}, period);
}

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
    double gait_duration_time_;
    double robot_width_;
    double robot_length_;
    double limb_length_hip_to_shoulder_;
    double limb_length_thigh_;
    double limb_length_shin_;

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

std::array<Eigen::Vector3d, 4> QuadrupedController::generateBasePosition(bool extra) const
{
  std::array<Eigen::Vector3d, 4> foot_targets;
  Eigen::Vector3d extra_vec = {extra ? 0.015 : 0.0, extra ? 0.025 : 0.0, 0.0};

  Eigen::Vector3d dims =
    Eigen::Vector3d(
      robot_length_ / 2.0,
      robot_width_ / 2.0 + limb_length_hip_to_shoulder_,
      -stand_height_
    )
    + extra_vec;

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
  double abs_max_vel = this->max_step_length_ / gait_duration_time_; 

  if (max_velocity > abs_max_vel) for (size_t i = 0; i < 4; i++) velocities[i] *= abs_max_vel / max_velocity;

  return velocities;
}

pond_result QuadrupedController::onStartupTF(const std::vector<void*>& args)
{
    timeout = parameter("timeout").asDouble().get(0.5);

    robot_width_                    = parameter("robot_width")                  .asDouble().get(0.07);
    robot_length_                   = parameter("robot_length")                 .asDouble().get(0.26);
    
    limb_length_hip_to_shoulder_    = parameter("limb_length_hip_to_shoulder")  .asDouble().get(0.065);
    limb_length_thigh_              = parameter("limb_length_thigh")            .asDouble().get(0.105);
    limb_length_shin_               = parameter("limb_length_shin")             .asDouble().get(0.115);

    step_height_                    = parameter("step_height")                  .asDouble().get(0.08);
    max_step_length_                = parameter("max_step_length")              .asDouble().get(0.10);
    stand_height_                   = parameter("stand_height")                 .asDouble().get(0.18);
    gait_duration_time_             = parameter("gait_duration_time")           .asDouble().get(1.5);
    
    gait_distributor = createDistributorTyped<std::vector<double>>({"gaits"});
    foot_position_distributor = createDistributorTyped<std::vector<FrameTransform>>({"foot_target_positions"});
    foot_positions_msg.resize(4);
    foot_positions_msg[0].stamp.frame_id = "base_link";

    std::array<Eigen::Vector2d, 4> foot_velocities = calculateFootVelocities(TwistCommand{.lin = {0.01, 0.0, 0.0}});

    for (uint32_t i = 0; i < 4; i++) trajectories[i].init(0.02, 0.1, 0.75, step_height_, 800.0, foot_velocities[i]);

    std::vector<std::string> joint_names = 
    {
        "FL", "FL_leg", "FL_foot",
        "FR", "FR_leg", "FR_foot",
        "BL", "BL_leg", "BL_foot",
        "BR", "BR_leg", "BR_foot",
    };

    joint_states.resize(12);
    pond::ChannelsInfo channels_info;

    for (uint32_t i = 0; i < 12; i++)
    {
        joint_states[i].joint_name = joint_names[i];
        channels_info.channel<MotorInterface>(joint_names[i] + "_motor/update");
    }

    update_receiver = createReceiver<MotorInterface>(channels_info, std::bind(&QuadrupedController::update_controller, this, std::placeholders::_1));

    locomotion_receiver = createReceiver<bool>({"enable_locomotion"}, [this](bool* status) {locomotion_activated.store(status);});    

    twist_receiver = createReceiver<TwistCommand>({"cmd_vel"}, [this](TwistCommand* new_twist){
        
        if (new_twist->stamp.time + 0.5 < pond::get_time()) return;

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
    double now = pond::get_time();

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

    std::vector<double> phase_array(4);

    bool twist_usable = twist_is_usable(twist_cmd_copy);

    if (twist_usable)
    {
        // Example: use fixed gait for now
        Gait advanced_gait = make_trot(1.5); // period = 1 second
        // AdvancedGait advanced_gait = AdvancedGait::make_trot(5.0);

        for (size_t i = 0; i < 4; i++) phase_array[i] = advanced_gait.get_phase(i, now);

    }
    else phase_array = {0.25, 0.25, 0.25, 0.25};
    
    gait_distributor.distribute(&phase_array);

    if (twist_cmd_copy.lin[0] != twist.lin[0] || twist_cmd_copy.lin[1] != twist.lin[1] || twist_cmd_copy.ang[2] != twist.ang[0])
    {
        // Generate base positions and foot velocity vectors
        std::array<Eigen::Vector2d, 4> foot_velocities = calculateFootVelocities(twist_usable ? twist_cmd_copy : TwistCommand{.lin = {0.01, 0.0, 0.0}});

        for (uint32_t i = 0; i < 4; i++) trajectories[i].init(0.02, 0.1, 0.75, step_height_, 800.0, foot_velocities[i]);

        twist = twist_cmd_copy;
    }

    std::array<Eigen::Vector3d, 4> base_positions = generateBasePosition(true);

    std::array<Eigen::Vector3d, 4> positions, velocities, accelerations;

    for(size_t i = 0; i < 4; i++)
    {
        trajectories[i].get_state(phase_array[i], positions[i], velocities[i], accelerations[i]);

        positions[i] += base_positions[i];

        foot_positions_msg[i].tf = Sophus::SE3d(Sophus::SO3d(), positions[i]);
        foot_positions_msg[i].stamp.time = now;
        foot_positions_msg[i].stamp.hw_time = now;

        double x = positions[i][0], y = positions[i][1], z = positions[i][2];

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

        ifs[3*i+0]->command.pos = hip_angle;
        ifs[3*i+1]->command.pos = shoulder_angle;
        ifs[3*i+2]->command.pos = knee_angle;
    }

    foot_position_distributor.distribute(&foot_positions_msg);
}
