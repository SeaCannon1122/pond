#include "pond/data_types/robot_state_types.hpp"
#include <rclcpp/executors.hpp>
#define POND_MODULE_CPP_MAKE_IMPLEMENTATION
#include "ros2_bridge.hpp"

template<typename pond_data_type, typename ros_msg>
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

template<typename pond_data_type, typename ros_msg>
struct ros_to_pond
{
    ~ros_to_pond() { distributor.destroy(); }

    std::shared_ptr<rclcpp::Subscription<ros_msg>> subscriber;
    pond::Distributor<pond_data_type> distributor;
};

class Ros2Bridge : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:

    template<typename pond_data_type, typename ros_msg>
    std::shared_ptr<pond_to_ros<pond_data_type, ros_msg>> create_pond_to_ros(const std::string& pond_topic, const std::string& ros_topic, void (*convert_function)(const pond_data_type&, ros_msg&), rclcpp::QoS& qos, bool vector)
    {
        std::shared_ptr<pond_to_ros<pond_data_type, ros_msg>> obj = std::make_shared<pond_to_ros<pond_data_type, ros_msg>>();

        obj->publisher = node->create_publisher<ros_msg>(ros_topic, qos);
        if (vector) obj->vector_receiver = createReceiver<std::vector<pond_data_type>>({pond_topic}, [pub_ptr = obj->publisher.get(), convert_function](std::vector<pond_data_type>* data)
        {
            ros_msg msg;
            for (const auto& d : *data)
            {
                convert_function(d, msg);
                pub_ptr->publish(msg);
            }
        });
        else obj->receiver = createReceiver<pond_data_type>({pond_topic}, [pub_ptr = obj->publisher.get(), convert_function](pond_data_type* data)
        {
            ros_msg msg;
            convert_function(*data, msg);
            pub_ptr->publish(msg);
        });

        obj->is_vector = vector;
        return obj;
    }

    template<typename pond_data_type, typename ros_msg>
    std::shared_ptr<ros_to_pond<pond_data_type, ros_msg>> create_ros_to_pond(const std::string& pond_topic, const std::string& ros_topic, void (*convert_function)(pond_data_type&, const ros_msg&), rclcpp::QoS& qos)
    {
        std::shared_ptr<ros_to_pond<pond_data_type, ros_msg>> obj = std::make_shared<ros_to_pond<pond_data_type, ros_msg>>();

        obj->distributor = createDistributor<pond_data_type>({pond_topic});
        obj->subscriber = node->create_subscription<ros_msg>(ros_topic, qos, [dis_ptr = &obj->distributor, convert_function](const std::shared_ptr<ros_msg> msg)
        {
            pond_data_type data;
            convert_function(data, *msg);
            dis_ptr->distribute(data);
        });

        return obj;
    }

    std::shared_ptr<rclcpp::Node> node;
    std::vector<std::shared_ptr<void>> bridges;
};

POND_MODULE_CPP_DECLARE(Ros2Bridge, "bridge", "bridging different message types to ros2")

POND_BUNDLE_DECLARE(
    "ros2 integration", 
    1,
    POND_MODULE(Ros2Bridge),
)

pond_result Ros2Bridge::onStartup(const std::vector<void*>& args)
{
    rclcpp::init(
        0, NULL, 
        rclcpp::InitOptions{},
        rclcpp::SignalHandlerOptions::None
    );
    node = std::make_shared<rclcpp::Node>(parameter("node_name").asString().get("pond_bridge"));

    int topic_count = parameter("topic_count").asInt().get(0);
    
    for (int i = 0; i < topic_count; i++)
    {
        std::string prefix = "topic" + std::to_string(i) + ".";

        auto direction_o = parameter(prefix+"direction").asString().getStrict({"POND_TO_ROS", "ROS_TO_POND"});
        auto pond_topic_o = parameter(prefix+"pond.topic").asString().getStrict();
        auto pond_type_o = parameter(prefix+"pond.type").asString().getStrict();
        auto ros_topic_o = parameter(prefix+"ros.topic").asString().getStrict();
        auto ros_type_o = parameter(prefix+"ros.type").asString().getStrict();
        if (!direction_o || !pond_topic_o || !pond_type_o || !ros_topic_o || !ros_type_o) continue;

        std::string direction = *direction_o, pond_topic = *pond_topic_o, pond_type = *pond_type_o, ros_topic = *ros_topic_o, ros_type = *ros_type_o;

        rclcpp::QoS qos(parameter(prefix+"ros.qos.depth").asInt().get(10));

        if (parameter(prefix+"ros.qos.reliable").asBool().get(true)) qos.reliable();
        else qos.best_effort();

        if (parameter(prefix+"ros.qos.volatile").asBool().get(true)) qos.durability_volatile();
        else qos.transient_local();

        if (direction == "POND_TO_ROS")
        {
            bool is_vector = parameter(prefix+"pond.is_vector").asBool().get(false);
            
            bool bridged = [&]() -> bool {
                if (pond_type == "ImuData")
                {
                    if (ros_type == "sensor_msgs::msg::Imu") bridges.emplace_back( create_pond_to_ros<ImuData, sensor_msgs::msg::Imu>(
                        pond_topic, ros_topic, ImuData__to__sensor_msgs_msg_Imu, qos, is_vector
                    ));
                    else return false;
                }
                else if (pond_type == "FrameTransform")
                {
                    if (ros_type == "geometry_msgs::msg::Pose") bridges.emplace_back(create_pond_to_ros<FrameTransform, geometry_msgs::msg::Pose>(
                        pond_topic, ros_topic, FrameTransform__to__geometry_msgs_msg_Pose, qos, is_vector
                    ));
                    else if (ros_type == "geometry_msgs::msg::PoseStamped") bridges.emplace_back(create_pond_to_ros<FrameTransform, geometry_msgs::msg::PoseStamped>(
                        pond_topic, ros_topic, FrameTransform__to__geometry_msgs_msg_PoseStamped, qos, is_vector
                    ));
                    else if (ros_type == "geometry_msgs::msg::TransformStamped") bridges.emplace_back(create_pond_to_ros<FrameTransform, geometry_msgs::msg::TransformStamped>(
                        pond_topic, ros_topic, FrameTransform__to__geometry_msgs_msg_TransformStamped, qos, is_vector
                    ));
                    else if (ros_type == "tf2_msgs::msg::TFMessage") bridges.emplace_back(create_pond_to_ros<FrameTransform, tf2_msgs::msg::TFMessage>(
                        pond_topic, ros_topic, FrameTransform__to__tf2_msgs_msg_TFMessage, qos, is_vector
                    ));
                    else return false;
                }
                else if (pond_type == "std::vector<FrameTransform>")
                {
                    if (ros_type == "tf2_msgs::msg::TFMessage") bridges.emplace_back(create_pond_to_ros<std::vector<FrameTransform>, tf2_msgs::msg::TFMessage>(
                        pond_topic, ros_topic, std_vector_FrameTransform__to__tf2_msgs_msg_TFMessage, qos, is_vector
                    ));
                    else return false;
                }
                else if (pond_type == "std::string")
                {
                    if (ros_type == "std_msgs::msg::String") bridges.emplace_back(create_pond_to_ros<std::string, std_msgs::msg::String>(
                        pond_topic, ros_topic, std_string__to__std_msgs_msg_String, qos, is_vector
                    ));
                    else return false;
                }
                else if (pond_type == "LaserScanSPtr")
                {
                    if (ros_type == "sensor_msgs::msg::LaserScan") bridges.emplace_back(create_pond_to_ros<LaserScanSPtr, sensor_msgs::msg::LaserScan>(
                        pond_topic, ros_topic, LaserScanSPtr__to__sensor_msgs_msg_LaserScan, qos, is_vector
                    ));
                    else return false;
                }
                else if (pond_type == "std::vector<JointState>")
                {
                    if (ros_type == "sensor_msgs::msg::JointState") bridges.emplace_back(create_pond_to_ros<std::vector<JointState>, sensor_msgs::msg::JointState>(
                        pond_topic, ros_topic, std_vector_JointState__to__sensor_msgs_msg_JointState, qos, is_vector
                    ));
                    else return false;
                }
                else if (pond_type == "CameraInfo")
                {
                    if (ros_type == "sensor_msgs::msg::CameraInfo") bridges.emplace_back(create_pond_to_ros<CameraInfo, sensor_msgs::msg::CameraInfo>(
                        pond_topic, ros_topic, CameraInfo__to__sensor_msgs_msg_CameraInfo, qos, is_vector
                    ));
                    else return false;
                }
                else return false;
    
                return true;
            }();

            if (bridged) POND_LOG("Bridging POND_TO_ROS:    '%s' '%s%s'  -->>  '%s' '%s'", pond_topic.c_str(), pond_type.c_str(), is_vector ? ".vector" : "", ros_topic.c_str(), ros_type.c_str());
            else POND_LOG("Cannot bridge POND_TO_ROS:    '%s' '%s%s'  -->>  '%s' '%s'", pond_topic.c_str(), pond_type.c_str(), is_vector ? ".vector" : "", ros_topic.c_str(), ros_type.c_str());
        }
        else
        {
            bool bridged = [&]() -> bool {
                if (pond_type == "TwistCommand")
                {
                    if (ros_type == "geometry_msgs::msg::Twist") bridges.emplace_back(create_ros_to_pond<TwistCommand, geometry_msgs::msg::Twist>(
                        pond_topic, ros_topic, geometry_msgs_msg_Twist__to__TwistCommand, qos
                    ));
                    else if (ros_type == "geometry_msgs::msg::TwistStamped") bridges.emplace_back(create_ros_to_pond<TwistCommand, geometry_msgs::msg::TwistStamped>(
                        pond_topic, ros_topic, geometry_msgs_msg_TwistStamped__to__TwistCommand, qos
                    ));
                    else return false;
                }
                else if (pond_type == "FrameTransform")
                {
                    if (ros_type == "geometry_msgs::msg::TransformStamped") bridges.emplace_back(create_ros_to_pond<FrameTransform, geometry_msgs::msg::TransformStamped>(
                        pond_topic, ros_topic, geometry_msgs_msg_TransformStamped__to__FrameTransform, qos
                    ));
                    else if (ros_type == "tf2_msgs::msg::TFMessage") bridges.emplace_back(create_ros_to_pond<FrameTransform, tf2_msgs::msg::TFMessage>(
                        pond_topic, ros_topic, tf2_msgs_msg_TFMessage__to__FrameTransform, qos
                    ));
                    else return false;
                }
                else if (pond_type == "std::vector<FrameTransform>")
                {
                    if (ros_type == "tf2_msgs::msg::TFMessage") bridges.emplace_back(create_ros_to_pond<std::vector<FrameTransform>, tf2_msgs::msg::TFMessage>(
                        pond_topic, ros_topic, tf2_msgs_msg_TFMessage__to__std_vector_FrameTransform, qos
                    ));
                    else return false;
                }
                else if (pond_type == "std::string")
                {
                    if (ros_type == "std_msgs::msg::String") bridges.emplace_back(create_ros_to_pond<std::string, std_msgs::msg::String>(
                        pond_topic, ros_topic, std_msgs_msg_String__to__std_string, qos
                    ));
                    else return false;
                }
                else if (pond_type == "LaserScanSPtr")
                {
                    if (ros_type == "sensor_msgs::msg::LaserScan") bridges.emplace_back(create_ros_to_pond<LaserScanSPtr, sensor_msgs::msg::LaserScan>(
                        pond_topic, ros_topic, sensor_msgs_msg_LaserScan__to__LaserScanSPtr, qos
                    ));
                    else return false;
                }
                else if (pond_type == "std::vector<JointState>")
                {
                    if (ros_type == "sensor_msgs::msg::JointState") bridges.emplace_back(create_ros_to_pond<std::vector<JointState>, sensor_msgs::msg::JointState>(
                        pond_topic, ros_topic, sensor_msgs_msg_JointState__to__std_vector_JointState, qos
                    ));
                    else return false;
                }
                else if (pond_type == "CameraInfo")
                {
                    if (ros_type == "sensor_msgs::msg::CameraInfo") bridges.emplace_back(create_ros_to_pond<CameraInfo, sensor_msgs::msg::CameraInfo>(
                        pond_topic, ros_topic, sensor_msgs_msg_CameraInfo__to__CameraInfo, qos
                    ));
                    else return false;
                }
                else return false;

                return true;
            }();

            if (bridged)  POND_LOG("Bridging ROS_TO_POND:    '%s' '%s'  -->>  '%s' '%s'", ros_topic.c_str(), ros_type.c_str(), pond_topic.c_str(), pond_type.c_str());
            else POND_LOG("Cannot bridge ROS_TO_POND:    '%s' '%s'  -->>  '%s' '%s'", ros_topic.c_str(), ros_type.c_str(), pond_topic.c_str(), pond_type.c_str());
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