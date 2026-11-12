from pond import Manager

from . import walk

class cfg:
    robot_state_bridges = [
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
        ####### joint_states ######
        {   "direction" : "POND_TO_ROS",

            "pond" : {"channel" : "set_robot_joints",           "type" : "std::vector<JointState>"},
            "ros"  : {"topic"   : "joint_states",               "type" : "sensor_msgs::msg::JointState"}  
        },
    ]
    cmd_bridges = [
        ######### cmd_vel #########
        {   "direction" : "ROS_TO_POND",

            "pond" : {"channel" : "cmd_vel",                    "type" : "TwistCommand"},
            "ros"  : {"topic"   : "cmd_vel_stamped",            "type" : "geometry_msgs::msg::TwistStamped",   "qos.depth" : 1, "qos.reliable" : False}  
        },
        ######## gait_type ########
        {   "direction" : "ROS_TO_POND",

            "pond" : {"channel" : "gait_type",                  "type" : "std::string"},
            "ros"  : {"topic"   : "gait_mode",                  "type" : "std_msgs::msg::String",   "qos.depth" : 1, "qos.reliable" : False}  
        },
        ###### gait_duration ######
        {   "direction" : "ROS_TO_POND",

            "pond" : {"channel" : "gait_duration",              "type" : "double"},
            "ros"  : {"topic"   : "gait_duration",              "type" : "std_msgs::msg::Float64",   "qos.depth" : 1, "qos.reliable" : False}  
        },
    ]
    debug_bridges = [
        ########## gaits ##########
        {   "direction" : "POND_TO_ROS", "rate" : int(0.1 / walk.cfg.frame_dt),

            "pond" : {"channel" : "gait_periods/out",           "type" : "std::vector<double>"},
            "ros"  : {"topic"   : "gait_periods",               "type" : "std_msgs::msg::Float64MultiArray"}  
        },
        #### stance and swing duration ####
        {   "direction" : "POND_TO_ROS", "rate" : int(0.2 / walk.cfg.frame_dt),

            "pond" : {"channel" : "partial_gait_durations/out", "type" : "std::vector<double>"},
            "ros"  : {"topic"   : "partial_gait_durations",     "type" : "std_msgs::msg::Float64MultiArray"}  
        },
        ###### gait_duration ######
        {   "direction" : "ROS_TO_POND",

            "pond" : {"channel" : "force",              "type" : "double"},
            "ros"  : {"topic"   : "force",              "type" : "std_msgs::msg::Float64"}  
        },
        ###### ground_forces ######
        {   "direction" : "POND_TO_ROS", "rate" : 10,

            "pond" : {"channel" : "ground_forces",                "type" : "std::vector<Eigen::Vector3d>"},
            "ros"  : {"topic"   : "ground_forces",                "type" : "geometry_msgs::msg::PoseArray"}  
        },
        ###### foot_target_poses ######
        {   "direction" : "POND_TO_ROS", "rate" : 10,

            "pond" : {"channel" : "foot_target_poses",            "type" : "std::vector<FrameTransform>"},
            "ros"  : {"topic"   : "foot_target_poses",            "type" : "geometry_msgs::msg::PoseArray"}  
        },
        ###### foot_poses ######
        {   "direction" : "POND_TO_ROS", "rate" : 10,

            "pond" : {"channel" : "foot_poses",                   "type" : "std::vector<FrameTransform>"},
            "ros"  : {"topic"   : "foot_poses",                   "type" : "geometry_msgs::msg::PoseArray"}  
        },
        ###### gait_duration ######
        {   "direction" : "POND_TO_ROS", "rate" : int(0.05 / walk.cfg.frame_dt),

            "pond" : {"channel" : "target_trajectory_states/out", "type" : "std::vector<TrajectoryState>"},
            "ros"  : {"topic"   : "target_trajectory_states",     "type" : "trajectory_msgs::msg::MultiDOFJointTrajectoryPoint"}  
        },
        ###### target_torques ######
        {   "direction" : "POND_TO_ROS", "rate" : 10,

            "pond" : {"channel" : "target_torques",              "type" : "std::vector<double>"},
            "ros"  : {"topic"   : "target_torques",              "type" : "std_msgs::msg::Float64MultiArray"}  
        },
    ]

def ros2_bridge(pm: Manager):

    thread_name = "ros2_bridge_thread"

    pm.set_thread_frame_time(thread_name, 0.010)

    pm.load_module(
        "ros2_bridge", "ros2/bridge", thread_name,
        {
            "node_name" : "pond_bridge",
            "bridges" : cfg.robot_state_bridges + cfg.cmd_bridges + cfg.debug_bridges
        },
    )