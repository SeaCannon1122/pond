#pragma once

#include "stamp.hpp"
#include <sophus/se3.hpp>

struct ImuData
{
    Stamp stamp;
    Eigen::Vector3d ang_vel;
    Eigen::Vector3d lin_acc;
};

struct ImuInfo
{
    Stamp stamp;
    uint32_t rate;

    struct
    {
        Eigen::Vector3d random_walk;
        Eigen::Vector3d noise;
        Eigen::Vector3d bias;
        Eigen::Matrix3d covariance;
    } ang_vel;

    struct
    {
        Eigen::Vector3d random_walk;
        Eigen::Vector3d noise;
        Eigen::Vector3d bias;
        Eigen::Matrix3d covariance;
    } lin_acc;
};