from pond import Manager

def ros2_bridge(pm: Manager):

    thread_name = "ros2_bridge_thread"

    pm.set_thread_frame_time(thread_name, 0.010)

    pm.load_module(
        "ros2_bridge", "ros2/bridge", thread_name,
        {
            "node_name" : "pond_bridge",
            "bridges" : [
                #### robot_description ####
                {   "direction" : "POND_TO_ROS",

                    "pond" : {"channel" : "robot_description",          "type" : "std::string"},
                    "ros"  : {"topic"   : "robot_description",          "type" : "std_msgs::msg::String",   "qos.depth" : 1, "qos.volatile" : False}  
                },
                ############ tf ###########
                {   "direction" : "POND_TO_ROS",

                    "pond" : {"channel" : "tf",                         "type" : "std::vector<FrameTransform>"},
                    "ros"  : {"topic"   : "tf",                         "type" : "tf2_msgs::msg::TFMessage"}  
                },
                ######## tf_static ########
                {   "direction" : "POND_TO_ROS",

                    "pond" : {"channel" : "tf_static",                  "type" : "std::vector<FrameTransform>"},
                    "ros"  : {"topic"   : "tf_static",                  "type" : "tf2_msgs::msg::TFMessage",   "qos.depth" : 1, "qos.volatile" : False}  
                },
                ######### cmd_vel #########
                {   "direction" : "ROS_TO_POND",

                    "pond" : {"channel" : "cmd_vel",                    "type" : "TwistCommand"},
                    "ros"  : {"topic"   : "cmd_vel",                    "type" : "geometry_msgs::msg::TwistStamped",   "qos.depth" : 1, "qos.reliable" : False}  
                },
                ####### joint_states ######
                {   "direction" : "POND_TO_ROS",

                    "pond" : {"channel" : "set_robot_joints",           "type" : "std::vector<JointState>"},
                    "ros"  : {"topic"   : "joint_states",               "type" : "sensor_msgs::msg::JointState"}  
                },
            ]
        },
    )