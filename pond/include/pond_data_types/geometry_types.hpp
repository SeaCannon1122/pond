#pragma once

#include "stamp.hpp"
#include <sophus/se3.hpp>

struct Point3d
{
    Stamp stamp;
    Eigen::Vector3d position;
};