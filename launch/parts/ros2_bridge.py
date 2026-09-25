from pond import Manager

def ros2_bridge(pm: Manager):

    pm.set_thread_frame_time("ros2_bridge_thread", 0.010)

    pm.load_module(
        name="ros2_bridge",
        bundle_name="ros2",
        module_name="bridge",
        thread_name="ros2_bridge_thread",
        parameters={
            "node_name" : "pond_bridge",
            "bridges" : [
                ####################
                {   "direction" : "POND_TO_ROS",

                    "pond" : {
                        "channel" : "robot_description",
                        "type" : "std::string",
                    },
                    "ros" : {
                        "topic" : "robot_description",
                        "type" : "std_msgs::msg::String",
                        "qos.depth" : 1,
                        "qos.volatile" : False,
                    }  
                },
                ####################
                {   "direction" : "POND_TO_ROS",

                    "pond" : {
                        "channel" : "tf",
                        "type" : "std::vector<FrameTransform>",
                    },
                    "ros" : {
                        "topic" : "tf",
                        "type" : "tf2_msgs::msg::TFMessage",
                    }  
                },
                ####################
                {   "direction" : "POND_TO_ROS",

                    "pond" : {
                        "channel" : "tf_static",
                        "type" : "std::vector<FrameTransform>",
                    },
                    "ros" : {
                        "topic" : "tf_static",
                        "type" : "tf2_msgs::msg::TFMessage",
                        "qos.depth" : 1,
                        "qos.volatile" : False,
                    }  
                },
                ####################
                {   "direction" : "ROS_TO_POND",

                    "pond" : {
                        "channel" : "cmd_vel",
                        "type" : "TwistCommand",
                    },
                    "ros" : {
                        "topic" : "/quac/cmd_vel_pilot",
                        "type" : "geometry_msgs::msg::TwistStamped",
                        "qos.reliable" : False,
                        "qos.depth" : 1,
                    }  
                },
                ####################
                {   "direction" : "POND_TO_ROS",

                    "pond" : {
                        "channel" : "scan",
                        "type" : "LaserScanSPtr",
                    },
                    "ros" : {
                        "topic" : "scan",
                        "type" : "sensor_msgs::msg::LaserScan",
                    }  
                },
                ####################
                {
                    "direction" : "POND_TO_ROS",

                    "pond" : {
                        "channel" : "set_robot_joints",
                        "type" : "std::vector<JointState>",
                    },

                    "ros" : {
                        "topic" : "joint_states",
                        "type" : "sensor_msgs::msg::JointState",
                    }  
                },
                ####################
                {   "direction" : "POND_TO_ROS",

                    "pond" : {
                        "channel" : "camera_front/color/info_reduced_rate",
                        "type" : "CameraInfo",
                    },
                    "ros" : {
                        "topic" : "/quac/camera_front/info",
                        "type" : "sensor_msgs::msg::CameraInfo",
                    }  
                },
                ####################
                {   "direction" : "POND_TO_ROS",

                    "pond" : {
                        "channel" : "camera_back/color/info_reduced_rate",
                        "type" : "CameraInfo",
                    },
                    "ros" : {
                        "topic" : "/quac/camera_back/info",
                        "type" : "sensor_msgs::msg::CameraInfo",
                    }  
                },
                ####################
                {   "direction" : "POND_TO_ROS",

                    "pond" : {
                        "channel" : "camera_gripper/color/info_reduced_rate",
                        "type" : "CameraInfo",
                    },
                    "ros" : {
                        "topic" : "/quac/camera_gripper/info",
                        "type" : "sensor_msgs::msg::CameraInfo",
                    }  
                },
                ####################
                {   "direction" : "ROS_TO_POND",

                    "pond" : {
                        "channel" : "gripper_width",
                        "type" : "double",
                    },
                    "ros" : {
                        "topic" : "/quac/gripper_width",
                        "type" : "std_msgs::msg::Float64",
                    }  
                },
                ####################
                {   "direction" : "ROS_TO_POND",

                    "pond" : {
                        "channel" : "arm_target",
                        "type" : "Pose2D",
                    },
                    "ros" : {
                        "topic" : "/quac/ee_pose",
                        "type" : "geometry_msgs::msg::Pose2D",
                    }  
                },
            ]
        },
    )

    pm.load_module(
        name="camera_front_info_rate_reducer",
        bundle_name="utility",
        module_name="channel_filter",
        thread_name="ros2_bridge_thread",
        parameters={
            "rate": 30,
            "channels_in": ["camera_front/color/info"],
            "channels_out" : ["camera_front/color/info_reduced_rate"],
        },
    )

    pm.load_module(
        name="camera_back_info_rate_reducer",
        bundle_name="utility",
        module_name="channel_filter",
        thread_name="ros2_bridge_thread",
        parameters={
            "rate": 30,
            "channels_in": ["camera_back/color/info"],
            "channels_out" : ["camera_back/color/info_reduced_rate"]
        },
    )

    pm.load_module(
        name="camera_gripper_info_rate_reducer",
        bundle_name="utility",
        module_name="channel_filter",
        thread_name="ros2_bridge_thread",
        parameters={
            "rate": 30,
            "channels_in": ["camera_gripper/color/info"],
            "channels_out" : ["camera_gripper/color/info_reduced_rate"]
        }
    )