#pragma once

#include <eigen3/Eigen/src/Core/Matrix.h>
#include "stamp.hpp"

struct BoundingBox
{
    Stamp stamp;

    Eigen::Vector2d corners[4];

    double confidence;
    std::string type; 
};
