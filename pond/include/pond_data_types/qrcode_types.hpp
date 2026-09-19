#pragma once

#include "video_types.hpp"

struct QRCodeDetection
{
    Eigen::Vector2d corners[4];
    std::string data; 
};

struct QRCodeDetectionRequest
{
    bool fulfilled = false;
    std::vector<QRCodeDetection> detections;
    ImgFrameSPtr image;
};