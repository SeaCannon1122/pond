#pragma once

#include <optional>
#include <stdint.h>
#include "transform_types.hpp"

struct GetFrameTransformRequest
{
    std::string source_frame;
    std::string target_frame;
    Sophus::SE3d tf;
    bool fulfilled = false;
    double time_point = 0.0;
};

struct GetJointInfoRequest
{
    std::string joint_name;
    bool fulfilled = false;

    Sophus::SE3d tf;
    bool is_static;
    std::optional<double> min_angle;
    std::optional<double> max_angle;
    std::string parent_link_name;
    std::string child_link_name;
};

struct JointState
{
    std::string joint_name = "";
    double time = 0;
    double hw_time = 0;
    double angle = 0;
};