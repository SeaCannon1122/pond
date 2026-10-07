#pragma once

#include "stamp.hpp"
#include <sophus/se3.hpp>

struct TwistCommand
{
    Stamp stamp;
    Eigen::Vector3d lin = {0.0, 0.0, 0.0};
    Eigen::Vector3d ang = {0.0, 0.0, 0.0};
};

struct Pose2D
{
    Stamp stamp;
    double x;
    double y;
    double theta;
};