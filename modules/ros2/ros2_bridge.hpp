#pragma once
#include "geometry_msgs/msg/transform.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <pond/pond.hpp>
#include <pond/data_types/imu_types.hpp>
#include <pond/data_types/command_types.hpp>
#include <pond/data_types/transform_types.hpp>
#include <pond/data_types/laser_scan_types.hpp>

static void std_string__to__std_msgs_msg_String(const std::string& pond, std_msgs::msg::String& ros)
{
    ros.data = pond;
}

static void std_msgs_msg_String__to__std_string(std::string& pond, const std_msgs::msg::String& ros)
{
    pond = ros.data;
}

static void ImuData__to__sensor_msgs_msg_Imu(const ImuData& pond, sensor_msgs::msg::Imu& ros)
{
    ros.header.frame_id = pond.stamp.frame_id;
    ros.header.stamp = rclcpp::Time(static_cast<int64_t>(pond.stamp.time * 1e9), RCL_SYSTEM_TIME);

    ros.angular_velocity.x = pond.ang_vel[0];
    ros.angular_velocity.y = pond.ang_vel[1];
    ros.angular_velocity.z = pond.ang_vel[2];

    ros.linear_acceleration.x = pond.lin_acc[0];
    ros.linear_acceleration.y = pond.lin_acc[1];
    ros.linear_acceleration.z = pond.lin_acc[2];
}

static void FrameTransform__to__geometry_msgs_msg_Pose(const FrameTransform& pond, geometry_msgs::msg::Pose& ros)
{
    Eigen::Vector3d t = pond.tf.translation();
    Eigen::Matrix3d R = pond.tf.rotationMatrix();
    
    ros.position.x = t.x();
    ros.position.y = t.y();
    ros.position.z = t.z();

    ros.orientation.x = std::atan2(R(2,1), R(2,2));
    ros.orientation.y = std::atan2(
        -R(2,0),
        std::sqrt(R(2,1) * R(2,1) + R(2,2) * R(2,2))
    );
    ros.orientation.z = std::atan2(R(1,0), R(0,0));
    ros.orientation.w = 1;
}

static void FrameTransform__to__geometry_msgs_msg_PoseStamped(const FrameTransform& pond, geometry_msgs::msg::PoseStamped& ros)
{
    FrameTransform__to__geometry_msgs_msg_Pose(pond, ros.pose);
    ros.header.frame_id = pond.stamp.frame_id;
    ros.header.stamp = rclcpp::Time(static_cast<int64_t>(pond.stamp.time * 1e9), RCL_SYSTEM_TIME);
}

static void FrameTransform__to__geometry_msgs_msg_TransformStamped(const FrameTransform& pond, geometry_msgs::msg::TransformStamped& ros)
{
    Eigen::Vector3d t = pond.tf.translation();
    Eigen::Quaterniond q = pond.tf.unit_quaternion();

    ros.transform.translation.x = t.x();
    ros.transform.translation.y = t.y();
    ros.transform.translation.z = t.z();

    ros.transform.rotation.x = q.x();
    ros.transform.rotation.y = q.y();
    ros.transform.rotation.z = q.z();
    ros.transform.rotation.w = q.w();

    ros.header.frame_id = pond.stamp.frame_id;
    ros.header.stamp = rclcpp::Time(static_cast<int64_t>(pond.stamp.time * 1e9), RCL_SYSTEM_TIME);
    ros.child_frame_id = pond.child_frame_id;
}

static void FrameTransform__to__tf2_msgs_msg_TFMessage(const FrameTransform& pond, tf2_msgs::msg::TFMessage& ros)
{
    ros.transforms.resize(1);
    FrameTransform__to__geometry_msgs_msg_TransformStamped(pond, ros.transforms[0]);
}

static void std_vector_FrameTransform__to__tf2_msgs_msg_TFMessage(const std::vector<FrameTransform>& pond, tf2_msgs::msg::TFMessage& ros)
{
    ros.transforms.resize(pond.size());
    for (int i = 0; i < pond.size(); i++) FrameTransform__to__geometry_msgs_msg_TransformStamped(pond[i], ros.transforms[i]);
}

static void geometry_msgs_msg_TransformStamped__to__FrameTransform(FrameTransform& pond, const geometry_msgs::msg::TransformStamped& ros)
{
    pond.stamp.frame_id = ros.header.frame_id;
    pond.stamp.time = rclcpp::Time(ros.header.stamp).seconds();
    pond.stamp.hw_time = pond.stamp.time;
    pond.child_frame_id = ros.child_frame_id;

    pond.tf = Sophus::SE3d(
        Eigen::Quaterniond(
            ros.transform.rotation.w,
            ros.transform.rotation.x,
            ros.transform.rotation.y,
            ros.transform.rotation.z
        ),
        Eigen::Vector3d (
            ros.transform.translation.x,
            ros.transform.translation.y,
            ros.transform.translation.z
        )
    );
}

static void tf2_msgs_msg_TFMessage__to__FrameTransform(FrameTransform& pond, const tf2_msgs::msg::TFMessage& ros)
{
    if (ros.transforms.size() > 0) geometry_msgs_msg_TransformStamped__to__FrameTransform(pond, ros.transforms[0]);
}

static void tf2_msgs_msg_TFMessage__to__std_vector_FrameTransform(std::vector<FrameTransform>& pond, const tf2_msgs::msg::TFMessage& ros)
{
    pond.resize(ros.transforms.size());
    for (int i = 0; i < ros.transforms.size(); i++) geometry_msgs_msg_TransformStamped__to__FrameTransform(pond[i], ros.transforms[i]);
}

static void geometry_msgs_msg_Twist__to__TwistCommand(TwistCommand& pond, const geometry_msgs::msg::Twist& ros)
{
    pond.stamp.frame_id = "base_link";
    pond.stamp.time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    pond.stamp.hw_time = pond.stamp.time;

    pond.lin[0] = ros.linear.x;
    pond.lin[1] = ros.linear.y;
    pond.lin[2] = ros.linear.z;

    pond.ang[0] = ros.angular.x;
    pond.ang[1] = ros.angular.y;
    pond.ang[2] = ros.angular.z;
}

static void geometry_msgs_msg_TwistStamped__to__TwistCommand(TwistCommand& pond, const geometry_msgs::msg::TwistStamped& ros)
{
    pond.stamp.frame_id = ros.header.frame_id;
    pond.stamp.time = rclcpp::Time(ros.header.stamp).seconds();
    pond.stamp.hw_time = pond.stamp.time;

    pond.lin[0] = ros.twist.linear.x;
    pond.lin[1] = ros.twist.linear.y;
    pond.lin[2] = ros.twist.linear.z;

    pond.ang[0] = ros.twist.angular.x;
    pond.ang[1] = ros.twist.angular.y;
    pond.ang[2] = ros.twist.angular.z;
}

static void sensor_msgs_msg_LaserScan__to__LaserScanSPtr(LaserScanSPtr& pond, const sensor_msgs::msg::LaserScan& ros)
{
    pond = std::make_shared<LaserScan>();

    pond->stamp.frame_id = ros.header.frame_id;
    pond->stamp.time = rclcpp::Time(ros.header.stamp).seconds();
    pond->stamp.hw_time = pond->stamp.time;

    pond->angle_min = ros.angle_min;
    pond->angle_max = ros.angle_max;
    pond->angle_increment = ros.angle_increment;
    pond->time_increment = ros.time_increment;
    pond->scan_time = ros.scan_time;
    pond->range_min = ros.range_min;
    pond->range_max = ros.range_max;
    pond->ranges = ros.ranges;
    pond->intensities = ros.intensities;
}

static void LaserScanSPtr__to__sensor_msgs_msg_LaserScan(const LaserScanSPtr& pond, sensor_msgs::msg::LaserScan& ros)
{
    ros.header.frame_id = pond->stamp.frame_id;
    ros.header.stamp = rclcpp::Time(static_cast<int64_t>(pond->stamp.time * 1e9), RCL_SYSTEM_TIME);

    ros.angle_min = pond->angle_min;
    ros.angle_max = pond->angle_max;
    ros.angle_increment = pond->angle_increment;
    ros.time_increment = pond->time_increment;
    ros.scan_time = pond->scan_time;
    ros.range_min = pond->range_min;
    ros.range_max = pond->range_max;
    ros.ranges = pond->ranges;
    ros.intensities = pond->intensities;
}