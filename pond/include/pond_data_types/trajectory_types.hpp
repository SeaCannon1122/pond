#pragma once

#include <eigen3/Eigen/src/Core/Matrix.h>
struct TrajectoryState
{
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
};