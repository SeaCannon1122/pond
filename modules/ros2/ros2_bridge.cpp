#include "ros2_bridge.hpp"
#include "mappings.hpp"

POND_MODULE_CPP_DECLARE(Ros2Bridge, "bridge", "bridging different message types to ros2")

POND_BUNDLE_DECLARE(
    "ros2 integration", 
    POND_MODULE(Ros2Bridge),
)

pond_result Ros2Bridge::onStartup(const std::vector<void*>& args)
{
    auto space = parameterSpace("bridges");
    uint32_t bridge_count = space.listLength("direction");
    if (bridge_count == 0) {POND_LOG("Error: Did not find any bridge definitions"); return POND_ERROR;}

    rclcpp::init(
        0, NULL, 
        rclcpp::InitOptions{},
        rclcpp::SignalHandlerOptions::None
    );
    node = std::make_shared<rclcpp::Node>(parameter("node_name").asString().get("pond_bridge"));

    for (int i = 0; i < bridge_count; i++)
    {
        auto direction_o = space.parameterAtIndex(i, "direction").asString().getStrict({"POND_TO_ROS", "ROS_TO_POND"});
        auto pond_channel_o = space.parameterAtIndex(i, "pond.channel").asString().getStrict();
        auto pond_type_o = space.parameterAtIndex(i, "pond.type").asString().getStrict();
        auto ros_type_o = space.parameterAtIndex(i, "ros.type").asString().getStrict();
        if (!direction_o || !pond_channel_o || !pond_type_o || !ros_type_o) continue;

        std::string direction = *direction_o, pond_channel = *pond_channel_o, pond_type = *pond_type_o, ros_type = *ros_type_o;
        std::string ros_topic = space.parameterAtIndex(i, "ros.topic").asString().get(pond_channel);

        rclcpp::QoS qos(space.parameterAtIndex(i, "ros.qos.depth").asInt().get(10));

        if (space.parameterAtIndex(i, "ros.qos.reliable").asBool().get(true)) qos.reliable(); else qos.best_effort();
        if (space.parameterAtIndex(i, "ros.qos.volatile").asBool().get(true)) qos.durability_volatile(); else qos.transient_local();

        if (direction == "POND_TO_ROS")
        {
            bool is_vector = space.parameterAtIndex(i, "pond.is_vector").asBool().get(false);
            
            #define IF_POND_TO_ROS(_pond_type, _ros_type, function) if (pond_type == #_pond_type && ros_type == #_ros_type) \
                { bridges.emplace_back( create_pond_to_ros<_pond_type, _ros_type>(pond_channel, ros_topic, function, qos, is_vector)); return true;}

            bool bridged = [&]() -> bool {

                IF_POND_TO_ROS(ImuData,                     sensor_msgs::msg::Imu,                  ImuData__to__sensor_msgs_msg_Imu)
                IF_POND_TO_ROS(FrameTransform,              geometry_msgs::msg::Pose,               FrameTransform__to__geometry_msgs_msg_Pose)
                IF_POND_TO_ROS(FrameTransform,              geometry_msgs::msg::PoseStamped,        FrameTransform__to__geometry_msgs_msg_PoseStamped)
                IF_POND_TO_ROS(FrameTransform,              geometry_msgs::msg::TransformStamped,   FrameTransform__to__geometry_msgs_msg_TransformStamped)
                IF_POND_TO_ROS(FrameTransform,              tf2_msgs::msg::TFMessage,               FrameTransform__to__tf2_msgs_msg_TFMessage)
                IF_POND_TO_ROS(std::vector<FrameTransform>, tf2_msgs::msg::TFMessage,               std_vector_FrameTransform__to__tf2_msgs_msg_TFMessage)
                IF_POND_TO_ROS(std::vector<FrameTransform>, geometry_msgs::msg::PoseArray,          std_vector_FrameTransform__to__geometry_msgs_msg_PoseArray)
                IF_POND_TO_ROS(std::string,                 std_msgs::msg::String,                  std_string__to__std_msgs_msg_String)
                IF_POND_TO_ROS(LaserScanSPtr,               sensor_msgs::msg::LaserScan,            LaserScanSPtr__to__sensor_msgs_msg_LaserScan)
                IF_POND_TO_ROS(std::vector<JointState>,     sensor_msgs::msg::JointState,           std_vector_JointState__to__sensor_msgs_msg_JointState)
                IF_POND_TO_ROS(CameraInfo,                  sensor_msgs::msg::CameraInfo,           CameraInfo__to__sensor_msgs_msg_CameraInfo)
                IF_POND_TO_ROS(double,                      std_msgs::msg::Float64,                 double__to__std_msgs_msg_Float64)
                IF_POND_TO_ROS(std::vector<double>,         std_msgs::msg::Float64MultiArray,       std_vector_double__to__std_msgs_msg_Float64MultiArray)
                IF_POND_TO_ROS(bool,                        std_msgs::msg::Bool,                    bool__to__std_msgs_msg_Bool)
                
                return false;
            }();

            if (bridged) POND_LOG("Bridging POND_TO_ROS:    '%s' '%s%s'  -->>  '%s' '%s'", pond_channel.c_str(), pond_type.c_str(), is_vector ? ".vector" : "", ros_topic.c_str(), ros_type.c_str());
            else POND_LOG("Cannot bridge POND_TO_ROS:    '%s' '%s%s'  -->>  '%s' '%s'", pond_channel.c_str(), pond_type.c_str(), is_vector ? ".vector" : "", ros_topic.c_str(), ros_type.c_str());
        }
        else
        {
            #define IF_ROS_TO_POND(_ros_type, _pond_type, function) if (pond_type == #_pond_type && ros_type == #_ros_type) \
                { bridges.emplace_back( create_ros_to_pond<_pond_type, _ros_type>(pond_channel, ros_topic, function, qos)); return true; }


            bool bridged = [&]() -> bool {

                IF_ROS_TO_POND(geometry_msgs::msg::Twist,               TwistCommand,                   geometry_msgs_msg_Twist__to__TwistCommand)
                IF_ROS_TO_POND(geometry_msgs::msg::TwistStamped,        TwistCommand,                   geometry_msgs_msg_TwistStamped__to__TwistCommand) 
                IF_ROS_TO_POND(geometry_msgs::msg::Pose2D,              Pose2D,                         geometry_msgs_msg_Pose2D__to__Pose2D)
                IF_ROS_TO_POND(geometry_msgs::msg::TransformStamped,    FrameTransform,                 geometry_msgs_msg_TransformStamped__to__FrameTransform)
                IF_ROS_TO_POND(geometry_msgs::msg::PoseStamped,         FrameTransform,                 geometry_msgs_msg_PoseStamped__to__FrameTransform)
                IF_ROS_TO_POND(tf2_msgs::msg::TFMessage,                FrameTransform,                 tf2_msgs_msg_TFMessage__to__FrameTransform)
                IF_ROS_TO_POND(tf2_msgs::msg::TFMessage,                std::vector<FrameTransform>,    tf2_msgs_msg_TFMessage__to__std_vector_FrameTransform)
                IF_ROS_TO_POND(std_msgs::msg::String,                   std::string,                    std_msgs_msg_String__to__std_string)
                IF_ROS_TO_POND(sensor_msgs::msg::LaserScan,             LaserScanSPtr,                  sensor_msgs_msg_LaserScan__to__LaserScanSPtr)
                IF_ROS_TO_POND(sensor_msgs::msg::JointState,            std::vector<JointState>,        sensor_msgs_msg_JointState__to__std_vector_JointState)
                IF_ROS_TO_POND(sensor_msgs::msg::CameraInfo,            CameraInfo,                     sensor_msgs_msg_CameraInfo__to__CameraInfo)
                IF_ROS_TO_POND(std_msgs::msg::Float64,                  double,                         std_msgs_msg_Float64__to__double)
                IF_ROS_TO_POND(std_msgs::msg::Float64MultiArray,        std::vector<double>,            std_msgs_msg_Float64MultiArray__to__std_vector_double)
                IF_ROS_TO_POND(std_msgs::msg::Bool,                     bool,                           std_msgs_msg_Bool__to__bool)
                
                return false;
            }();

            if (bridged)  POND_LOG("Bridging ROS_TO_POND:    '%s' '%s'  -->>  '%s' '%s'", ros_topic.c_str(), ros_type.c_str(), pond_channel.c_str(), pond_type.c_str());
            else POND_LOG("Cannot bridge ROS_TO_POND:    '%s' '%s'  -->>  '%s' '%s'", ros_topic.c_str(), ros_type.c_str(), pond_channel.c_str(), pond_type.c_str());
        }
    }

    return POND_SUCCESS;
}

void Ros2Bridge::onShutdown()
{
    bridges.resize(0);
    node.reset();
    rclcpp::shutdown();
}

void Ros2Bridge::onFrame()
{
    rclcpp::spin_some(node);
}