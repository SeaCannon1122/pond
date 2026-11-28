#pragma once

#include "stamp.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <sophus/se3.hpp>
struct TrajectoryPoint
{
    Stamp stamp;
    bool use_rpy = false;
    Sophus::SE3d pose;
    Eigen::Vector3d rpy{0,0,0};
    Eigen::Vector3d lin_vel{0,0,0}, ang_vel{0,0,0};
    Eigen::Vector3d lin_acc{0,0,0}, ang_acc{0,0,0};
};