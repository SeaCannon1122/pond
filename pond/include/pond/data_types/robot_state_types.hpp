#pragma once

#include <stdint.h>
#include "transform_types.hpp"

struct GetFrameTransformRequest
{
    std::string source_frame;
    std::string target_frame;
    Sophus::SE3d tf;
    bool fufilled = false;
    double time_point = 0.0;
};

struct GetJointInfoRequest
{
    std::string joint_name;
    bool fufilled = false;

    Sophus::SE3d tf;
    bool is_static;
    std::string parent_link_name;
    std::string child_link_name;
};

struct JointState
{
    std::string joint_name;
    double angle;
};