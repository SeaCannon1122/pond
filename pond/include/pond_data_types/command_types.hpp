#pragma once

#include "stamp.hpp"
#include <sophus/se3.hpp>

struct TwistCommand
{
    Stamp stamp;
    Eigen::Vector3d lin;
    Eigen::Vector3d ang;
};

struct Pose2D
{
    Stamp stamp;
    double x;
    double y;
    double theta;
};