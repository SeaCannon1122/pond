#pragma once

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_msgs/msg/tf_message.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <pond/pond.hpp>

#include <pond_data_types/video_types.hpp>
#include <pond_data_types/imu_types.hpp>
#include <pond_data_types/command_types.hpp>
#include <pond_data_types/transform_types.hpp>
#include <pond_data_types/laser_scan_types.hpp>
#include <pond_data_types/robot_state_types.hpp>

void ImuData__to__sensor_msgs_msg_Imu(const ImuData& pond, sensor_msgs::msg::Imu& ros);

void FrameTransform__to__geometry_msgs_msg_Pose(const FrameTransform& pond, geometry_msgs::msg::Pose& ros);

void geometry_msgs_msg_PoseStamped__to__FrameTransform(FrameTransform& pond, const geometry_msgs::msg::PoseStamped& ros);

void std_vector_FrameTransform__to__geometry_msgs_msg_PoseArray(const std::vector<FrameTransform>& pond, geometry_msgs::msg::PoseArray& ros);

void FrameTransform__to__geometry_msgs_msg_PoseStamped(const FrameTransform& pond, geometry_msgs::msg::PoseStamped& ros);

void FrameTransform__to__geometry_msgs_msg_TransformStamped(const FrameTransform& pond, geometry_msgs::msg::TransformStamped& ros);

void FrameTransform__to__tf2_msgs_msg_TFMessage(const FrameTransform& pond, tf2_msgs::msg::TFMessage& ros);

void std_vector_FrameTransform__to__tf2_msgs_msg_TFMessage(const std::vector<FrameTransform>& pond, tf2_msgs::msg::TFMessage& ros);

void geometry_msgs_msg_TransformStamped__to__FrameTransform(FrameTransform& pond, const geometry_msgs::msg::TransformStamped& ros);

void tf2_msgs_msg_TFMessage__to__FrameTransform(FrameTransform& pond, const tf2_msgs::msg::TFMessage& ros);

void tf2_msgs_msg_TFMessage__to__std_vector_FrameTransform(std::vector<FrameTransform>& pond, const tf2_msgs::msg::TFMessage& ros);

void geometry_msgs_msg_Pose2D__to__Pose2D(Pose2D& pond, const geometry_msgs::msg::Pose2D& ros);

void geometry_msgs_msg_Twist__to__TwistCommand(TwistCommand& pond, const geometry_msgs::msg::Twist& ros);
void geometry_msgs_msg_TwistStamped__to__TwistCommand(TwistCommand& pond, const geometry_msgs::msg::TwistStamped& ros);

void sensor_msgs_msg_LaserScan__to__LaserScanSPtr(LaserScanSPtr& pond, const sensor_msgs::msg::LaserScan& ros);
void LaserScanSPtr__to__sensor_msgs_msg_LaserScan(const LaserScanSPtr& pond, sensor_msgs::msg::LaserScan& ros);

void sensor_msgs_msg_JointState__to__std_vector_JointState(std::vector<JointState>& pond, const sensor_msgs::msg::JointState& ros);
void std_vector_JointState__to__sensor_msgs_msg_JointState(const std::vector<JointState>& pond, sensor_msgs::msg::JointState& ros);

void CameraInfo__to__sensor_msgs_msg_CameraInfo(const CameraInfo& pond, sensor_msgs::msg::CameraInfo& ros);
void sensor_msgs_msg_CameraInfo__to__CameraInfo(CameraInfo& pond, const sensor_msgs::msg::CameraInfo& ros);

void std_string__to__std_msgs_msg_String(const std::string& pond, std_msgs::msg::String& ros);
void std_msgs_msg_String__to__std_string(std::string& pond, const std_msgs::msg::String& ros);

void std_msgs_msg_Bool__to__bool(bool& pond, const std_msgs::msg::Bool& ros);
void bool__to__std_msgs_msg_Bool(const bool& pond, std_msgs::msg::Bool& ros);

void std_msgs_msg_Float64__to__double(double& pond, const std_msgs::msg::Float64& ros);
void double__to__std_msgs_msg_Float64(const double& pond, std_msgs::msg::Float64& ros);

void std_msgs_msg_Float64MultiArray__to__std_vector_double(std::vector<double>& pond, const std_msgs::msg::Float64MultiArray& ros);
void std_vector_double__to__std_msgs_msg_Float64MultiArray(const std::vector<double>& pond, std_msgs::msg::Float64MultiArray& ros);