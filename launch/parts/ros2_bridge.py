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
                    "ros"  : {"topic"   : "quac/cmd_vel_pilot",        "type" : "geometry_msgs::msg::TwistStamped",   "qos.depth" : 1, "qos.reliable" : False}  
                },
                ########### scan ##########
                {   "direction" : "POND_TO_ROS",

                    "pond" : {"channel" : "scan",                       "type" : "LaserScanSPtr"},
                    "ros"  : {"topic"   : "scan",                       "type" : "sensor_msgs::msg::LaserScan"}  
                },
                ####### joint_states ######
                {   "direction" : "POND_TO_ROS",

                    "pond" : {"channel" : "set_robot_joints",           "type" : "std::vector<JointState>"},
                    "ros"  : {"topic"   : "quac/joint_states",          "type" : "sensor_msgs::msg::JointState"}  
                },
                ####### gripper_width #####
                {   "direction" : "ROS_TO_POND",

                    "pond" : {"channel" : "gripper_width",              "type" : "double"},
                    "ros"  : {"topic"   : "quac/gripper_width",         "type" : "std_msgs::msg::Float64"}  
                },
                ######## arm_target #######
                {   "direction" : "ROS_TO_POND",

                    "pond" : {"channel" : "arm_target",                 "type" : "Pose2D"},
                    "ros"  : {"topic"   : "quac/ee_pose",               "type" : "geometry_msgs::msg::Pose2D"}  
                },

                #### camera_front_info ####
                {   "direction" : "POND_TO_ROS",

                    "pond" : {"channel" : "camera_front/color/info_rr",         "type" : "CameraInfo"},
                    "ros"  : {"topic"   : "quac/camera_front/info",             "type" : "sensor_msgs::msg::CameraInfo"}  
                },
                ##### camera_back_info ####
                {   "direction" : "POND_TO_ROS",

                    "pond" : {"channel" : "camera_back/color/info_rr",          "type" : "CameraInfo"},
                    "ros"  : {"topic"   : "quac/camera_back/info",              "type" : "sensor_msgs::msg::CameraInfo"}  
                },
                ### camera_gripper_info ###
                {   "direction" : "POND_TO_ROS",

                    "pond" : {"channel" : "camera_gripper/color/info_rr",       "type" : "CameraInfo"},
                    "ros"  : {"topic"   : "quac/camera_gripper/info",           "type" : "sensor_msgs::msg::CameraInfo"}
                },
            ]
        },
    )

    pm.load_module(
        "camera_front_info_rate_reducer", "utility/channel_filter", thread_name,
        {
            "rate": 30,
            "channels_in":   ["camera_front/color/info"],
            "channels_out" : ["camera_front/color/info_rr"],
        },
    )

    pm.load_module(
        "camera_back_info_rate_reducer", "utility/channel_filter", thread_name,
        {
            "rate": 30,
            "channels_in":   ["camera_back/color/info"],
            "channels_out" : ["camera_back/color/info_rr"],
        },
    )

    pm.load_module(
        "camera_gripper_info_rate_reducer", "utility/channel_filter", thread_name,
        {
            "rate": 30,
            "channels_in":   ["camera_gripper/color/info"],
            "channels_out" : ["camera_gripper/color/info_rr"],
        },
    )