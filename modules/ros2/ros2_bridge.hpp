#pragma once

#include <pond/pond.hpp>
#include <rclcpp/rclcpp.hpp>

template<typename pond_data_type, typename ros_msg>
struct pond_to_ros
{
    ~pond_to_ros() { receiver.destroy();}

    std::shared_ptr<rclcpp::Publisher<ros_msg>> publisher;
    pond::Receiver receiver;
    bool is_vector;
};

template<typename pond_data_type, typename ros_msg>
struct ros_to_pond
{
    ~ros_to_pond() { distributor.destroy(); }

    std::shared_ptr<rclcpp::Subscription<ros_msg>> subscriber;
    pond::DistributorTyped<pond_data_type> distributor;
};

class Ros2Bridge : public pond::ModuleBase
{
public:
    virtual pond_result onStartup(const std::vector<void*>& args) override;
    virtual void onShutdown() override;
    virtual void onFrame() override;
private:

    template<typename pond_data_type, typename ros_msg>
    std::shared_ptr<pond_to_ros<pond_data_type, ros_msg>> create_pond_to_ros(const std::string& pond_channel, const std::string& ros_topic, void (*convert_function)(const pond_data_type&, ros_msg&), rclcpp::QoS& qos, bool vector)
    {
        std::shared_ptr<pond_to_ros<pond_data_type, ros_msg>> obj = std::make_shared<pond_to_ros<pond_data_type, ros_msg>>();

        obj->publisher = node->create_publisher<ros_msg>(ros_topic, qos);
        if (vector) obj->receiver = createReceiver<std::vector<pond_data_type>>({pond_channel}, [pub_ptr = obj->publisher.get(), convert_function](std::vector<pond_data_type>* data)
        {
            ros_msg msg;
            for (const auto& d : *data)
            {
                convert_function(d, msg);
                pub_ptr->publish(msg);
            }
        });
        else obj->receiver = createReceiver<pond_data_type>({pond_channel}, [pub_ptr = obj->publisher.get(), convert_function](pond_data_type* data)
        {
            ros_msg msg;
            convert_function(*data, msg);
            pub_ptr->publish(msg);
        });

        obj->is_vector = vector;
        return obj;
    }

    template<typename pond_data_type, typename ros_msg>
    std::shared_ptr<ros_to_pond<pond_data_type, ros_msg>> create_ros_to_pond(const std::string& pond_channel, const std::string& ros_topic, void (*convert_function)(pond_data_type&, const ros_msg&), rclcpp::QoS& qos)
    {
        std::shared_ptr<ros_to_pond<pond_data_type, ros_msg>> obj = std::make_shared<ros_to_pond<pond_data_type, ros_msg>>();

        obj->distributor = createDistributorTyped<pond_data_type>({pond_channel});
        obj->subscriber = node->create_subscription<ros_msg>(ros_topic, qos, [dis_ptr = &obj->distributor, convert_function](const std::shared_ptr<ros_msg> msg)
        {
            pond_data_type data;
            convert_function(data, *msg);
            dis_ptr->distribute(&data);
        });

        return obj;
    }

    std::shared_ptr<rclcpp::Node> node;
    std::vector<std::shared_ptr<void>> bridges;
};