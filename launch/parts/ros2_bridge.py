from pond import Manager

def ros2_bridge(pm: Manager):

    pm.load_module(
        name="ros2_bridge_frame_timer",
        bundle_name="utility",
        module_name="frame_timer",
        thread_name="ros2_bridge_thread",
        parameters={"min_time" : 0.010},
        topic_mappings={},
        topic_namespace=""
    )

    pm.load_module(
        name="ros2_bridge",
        bundle_name="ros2",
        module_name="bridge",
        thread_name="ros2_bridge_thread",
        parameters={
            "node_name" : "pond_bridge",
            "topic_count" : 9,

            ####################
            "topic0" : {
                "direction" : "POND_TO_ROS",

                "pond" : {
                    "topic" : "robot_description",
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
            "topic1" : {
                "direction" : "POND_TO_ROS",

                "pond" : {
                    "topic" : "tf",
                    "type" : "std::vector<FrameTransform>",
                },

                "ros" : {
                    "topic" : "tf",
                    "type" : "tf2_msgs::msg::TFMessage",
                }  
            },

            ####################
            "topic2" : {
                "direction" : "POND_TO_ROS",

                "pond" : {
                    "topic" : "tf_static",
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
            "topic3" : {
                "direction" : "ROS_TO_POND",

                "pond" : {
                    "topic" : "cmd_vel",
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
            "topic4" : {
                "direction" : "POND_TO_ROS",

                "pond" : {
                    "topic" : "scan",
                    "type" : "LaserScanSPtr",
                },

                "ros" : {
                    "topic" : "scan",
                    "type" : "sensor_msgs::msg::LaserScan",
                }  
            },

            ####################
            "topic5" : {
                "direction" : "POND_TO_ROS",

                "pond" : {
                    "topic" : "joint_states",
                    "type" : "std::vector<JointState>",
                },

                "ros" : {
                    "topic" : "joint_states",
                    "type" : "sensor_msgs::msg::JointState",
                }  
            },

            ####################
            "topic6" : {
                "direction" : "POND_TO_ROS",

                "pond" : {
                    "topic" : "camera_front/color/cam_info_reduced_rate",
                    "type" : "CameraInfo",
                },

                "ros" : {
                    "topic" : "/quac/camera_front/info",
                    "type" : "sensor_msgs::msg::CameraInfo",
                }  
            },

            ####################
            "topic7" : {
                "direction" : "POND_TO_ROS",

                "pond" : {
                    "topic" : "camera_back/color/cam_info_reduced_rate",
                    "type" : "CameraInfo",
                },

                "ros" : {
                    "topic" : "/quac/camera_back/info",
                    "type" : "sensor_msgs::msg::CameraInfo",
                }  
            },

            ####################
            "topic8" : {
                "direction" : "POND_TO_ROS",

                "pond" : {
                    "topic" : "camera_gripper/color/cam_info_reduced_rate",
                    "type" : "CameraInfo",
                },

                "ros" : {
                    "topic" : "/quac/camera_gripper/info",
                    "type" : "sensor_msgs::msg::CameraInfo",
                }  
            },
        },
        topic_mappings={},
        topic_namespace=""
    )

    pm.load_module(
        name="camera_front_info_rate_reducer",
        bundle_name="utility",
        module_name="topic_filter",
        thread_name="ros2_bridge_thread",
        parameters={
            "rate": 30,
            "topics_in": {"camera_front/color/cam_info", "a"},
            "topics_out" : {"camera_front/color/cam_info_reduced_rate", "b"}
        },
        topic_mappings={},
        topic_namespace=""
    )

    # pm.load_module(
    #     name="camera_back_info_rate_reducer",
    #     bundle_name="utility",
    #     module_name="topic_filter",
    #     thread_name="ros2_bridge_thread",
    #     parameters={
    #         "rate": 30,
    #         "topics_in": {"camera_back/color/cam_info"},
    #         "topics_out" : {"camera_back/color/cam_info_reduced_rate"}
    #     },
    #     topic_mappings={},
    #     topic_namespace=""
    # )

    # pm.load_module(
    #     name="camera_gripper_info_rate_reducer",
    #     bundle_name="utility",
    #     module_name="topic_filter",
    #     thread_name="ros2_bridge_thread",
    #     parameters={
    #         "rate": 30,
    #         "topics_in": {"camera_gripper/color/cam_info"},
    #         "topics_out" : {"camera_gripper/color/cam_info_reduced_rate"}
    #     },
    #     topic_mappings={},
    #     topic_namespace=""
    # )