#pragma once

#include "stamp.hpp"
#include "geometry_types.hpp"

struct ImuData
{
    Point lin_acc;
    Point ang_vel;
};

struct ImuDataStamped
{
    ImuData data;
    Stamp stamp;
};