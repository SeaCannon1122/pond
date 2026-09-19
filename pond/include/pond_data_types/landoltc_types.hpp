#pragma once

#include "video_types.hpp"

struct Landoltc
{
    enum class Orientation 
    {
        UP = 0,
        UP_LEFT = 1,
        LEFT = 2,
        DOWN_LEFT = 3,
        DOWN = 4,
        DOWN_RIGHT = 5,
        RIGHT = 6,
        UP_RIGHT = 7,
        Direction_count = 8
    };

};

struct LandoltcDetection
{
    Eigen::Vector2d midpoint;
    double radius;
    std::array<Eigen::Vector2d, 2> opening;
};

struct LandoltcDetectionRequest
{
    bool fulfilled = false;
    std::vector<LandoltcDetection> detections;
    ImgFrameSPtr image;
};