#pragma once

#include "stamp.hpp"

struct Point
{
    double x;
    double y;
    double z;
    double w;
};

struct PointStamped
{
    Stamp stamp;
    Point point;
};

struct Pose
{
    Point p;
    Point o;
};

struct PoseStamped
{
    Stamp stamp;
    Pose pose;
};
