#pragma once

#include "stamp.hpp"
#include <sophus/se3.hpp>

struct TwistCommand
{
    Stamp stamp;
    Eigen::Vector3d lin;
    Eigen::Vector3d ang;
};