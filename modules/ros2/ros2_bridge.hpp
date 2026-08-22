#pragma once
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <pond/pond.hpp>
#include <pond/data_types/imu_types.hpp>
#include <pond/data_types/command_types.hpp>
#include <pond/data_types/transform_types.hpp>

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
    ros.header.frame_id = pond.parent_frame_id;
    ros.header.stamp = rclcpp::Time(static_cast<int64_t>(pond.stamp.time * 1e9), RCL_SYSTEM_TIME);
}

static void geometry_msgs_msg_Twist__to__TwistCommand(TwistCommand& pond, const geometry_msgs::msg::Twist& ros)
{
    pond.stamp.frame_id = "base_link";
    pond.stamp.time = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    
    pond.lin[0] = ros.linear.x;
    pond.lin[1] = ros.linear.y;
    pond.lin[2] = ros.linear.z;

    pond.ang[0] = ros.angular.x;
    pond.ang[1] = ros.angular.y;
    pond.ang[2] = ros.angular.z;
}

static void geometry_msgs_msg_TwistStamped__to__TwistCommand(TwistCommand& pond, const geometry_msgs::msg::TwistStamped& ros)
{
    geometry_msgs_msg_Twist__to__TwistCommand(pond, ros.twist);
    pond.stamp.frame_id = ros.header.frame_id;
    pond.stamp.time = rclcpp::Time(ros.header.stamp).seconds();
}