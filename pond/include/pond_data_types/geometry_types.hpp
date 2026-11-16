#pragma once

#include "stamp.hpp"
#include <sophus/se3.hpp>

struct Point3d
{
    Stamp stamp;
    Eigen::Vector3d position;
};

struct FrameState
{
    Stamp stamp;
    Sophus::SE3d pose;
    Eigen::Vector3d lin_vel, ang_vel;
};