#include "pond/pond.h"
#include <pond/data_types/imu_types.hpp>
#include <memory>
#include <pond/pond.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

template<typename ros_msg, typename pond_data_type>
struct pond_to_ros
{
    ~pond_to_ros()
    {
        if (is_vector) vector_receiver.destroy();
        else receiver.destroy();
    }

    std::shared_ptr<rclcpp::Publisher<ros_msg>> publisher;
    pond::Receiver<pond_data_type> receiver;
    pond::Receiver<std::vector<pond_data_type>> vector_receiver;
    bool is_vector;
};

class Ros2Bridge : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:
    template<typename ros_msg, typename pond_data_type>

    void create_pond_to_ros(const std::string& pond_topic, std::shared_ptr<pond_to_ros<ros_msg, pond_data_type>>& obj, void (*convert_function)(const pond_data_type&, ros_msg&), bool vector)
    {
        rclcpp::QoS qos(parameter(pond_topic+".ros.qos.depth").asInt().get(10));
        
        if (parameter(pond_topic+".ros.qos.reliable").asBool().get(true)) qos.reliable();
        else qos.best_effort();

        obj->publisher = node->create_publisher<ros_msg>(parameter(pond_topic+".ros.topic").asString().get("ros_topic"), qos);
        if (vector) obj->vector_receiver = createReceiver<std::vector<pond_data_type>>({pond_topic}, [pub_ptr = obj->publisher.get(), convert_function](std::vector<pond_data_type>& data)
        {
            ros_msg msg;
            for (const auto& d : data)
            {
                convert_function(d, msg);
                pub_ptr->publish(msg);
            }
        });
        else obj->receiver = createReceiver<pond_data_type>({pond_topic}, [pub_ptr = obj->publisher.get(), convert_function](pond_data_type& data)
        {
            ros_msg msg;
            convert_function(data, msg);
            pub_ptr->publish(msg);
        });

        obj->is_vector = vector;
    }

    std::shared_ptr<rclcpp::Node> node;
    std::vector<std::shared_ptr<void>> pond_to_ros_s;
};

POND_MODULE_CPP_DECLARE(Ros2Bridge, "ros2_bridge", "bridging different message types to ros2")

void pond_to_ros_convert_ImuDataStamped(const ImuDataStamped& data, sensor_msgs::msg::Imu& msg)
{
    msg.header.frame_id = data.stamp.frame_id;
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(data.stamp.time * 1e9), RCL_SYSTEM_TIME);

    msg.angular_velocity.x = data.data.ang_vel.x;
    msg.angular_velocity.y = data.data.ang_vel.y;
    msg.angular_velocity.z = data.data.ang_vel.z;

    msg.linear_acceleration.x = data.data.lin_acc.x;
    msg.linear_acceleration.y = data.data.lin_acc.y;
    msg.linear_acceleration.z = data.data.lin_acc.z;
}

void pond_to_ros_convert_PoseStamped(const PoseStamped& data, geometry_msgs::msg::PoseStamped& msg)
{
    msg.header.frame_id = data.stamp.frame_id;
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(data.stamp.time * 1e9), RCL_SYSTEM_TIME);

    msg.pose.position.x = data.pose.p.x;
    msg.pose.position.y = data.pose.p.y;
    msg.pose.position.z = data.pose.p.z;

    msg.pose.orientation.x = data.pose.o.x;
    msg.pose.orientation.y = data.pose.o.y;
    msg.pose.orientation.z = data.pose.o.z;
    msg.pose.orientation.w = data.pose.o.w;
}

pond_result Ros2Bridge::onStartup(const std::vector<void*>& args)
{
    rclcpp::init(
        0, NULL, 
        rclcpp::InitOptions{},
        rclcpp::SignalHandlerOptions::None
    );
    node = std::make_shared<rclcpp::Node>(parameter("node_name").asString().get("pond_bridge"));

    POND_LOG("POND_TO_ROS:");
    auto pond_topics = parameter("pond_topics").asStringArray().getStrict();
    if (!pond_topics) return POND_ERROR;
    
    for (const auto& pond_topic : *pond_topics)
    {
        POND_LOG("  %s -> %s", pond_topic.c_str(), parameter(pond_topic+".ros.topic").asString().get("ros_topic").c_str());

        auto pond_type_full = parameter(pond_topic+".pond.type").asString().getStrict();
        if (!pond_type_full) continue;
        
        std::string pond_type = *pond_type_full;
        std::string suffix = "";

        if (auto pos = (*pond_type_full).find('.'); pos != std::string::npos)
        {
            pond_type = (*pond_type_full).substr(0, pos);
            suffix  = (*pond_type_full).substr(pos + 1);
        }

        bool is_vector = (suffix == "Vector");

        if (pond_type == "ImuDataStamped")
        {
            auto obj = std::make_shared<pond_to_ros<sensor_msgs::msg::Imu, ImuDataStamped>>();
            create_pond_to_ros<sensor_msgs::msg::Imu, ImuDataStamped>(pond_topic, obj, pond_to_ros_convert_ImuDataStamped, is_vector);
            pond_to_ros_s.emplace_back(obj);
        }
        else if (pond_type == "PoseStamped")
        {
            auto obj = std::make_shared<pond_to_ros<geometry_msgs::msg::PoseStamped, PoseStamped>>();
            create_pond_to_ros<geometry_msgs::msg::PoseStamped, PoseStamped>(pond_topic, obj, pond_to_ros_convert_PoseStamped, is_vector);
            pond_to_ros_s.emplace_back(obj);
        }
        else POND_LOG("Cannot bridge pond topic type '%s'", pond_type.c_str());
       
    }

    return POND_SUCCESS;
}

void Ros2Bridge::onShutdown()
{
    pond_to_ros_s.resize(0);
    node.reset();
    rclcpp::shutdown();
}

void Ros2Bridge::onFrame()
{
}
