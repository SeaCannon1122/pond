#pragma once

#include "stamp.hpp"
#include <eigen3/Eigen/src/Core/Matrix.h>

struct Point3d
{
    Stamp stamp;
    Eigen::Vector3d position;
};