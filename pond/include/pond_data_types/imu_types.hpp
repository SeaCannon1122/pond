#pragma once

#include "stamp.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <sophus/se3.hpp>

struct ImuData
{
    Stamp stamp;
    Eigen::Vector3d ang_vel;
    Eigen::Vector3d lin_acc;
    Sophus::SO3d orientation;
};

struct ImuInfo
{
    Stamp stamp;
    uint32_t rate = 0;

    struct
    {
        Eigen::Vector3d random_walk = Eigen::Vector3d::Zero();
        Eigen::Vector3d noise = Eigen::Vector3d::Zero();
        Eigen::Vector3d bias = Eigen::Vector3d::Zero();
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    } ang_vel;

    struct
    {
        Eigen::Vector3d random_walk = Eigen::Vector3d::Zero();
        Eigen::Vector3d noise = Eigen::Vector3d::Zero();
        Eigen::Vector3d bias = Eigen::Vector3d::Zero();
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    } lin_acc;
};