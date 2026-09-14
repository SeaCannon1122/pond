#pragma once

#include "stamp.hpp"
#include <sophus/se3.hpp>

struct FrameTransform
{
    Stamp stamp;
    std::string child_frame_id;
    Sophus::SE3d tf;
};

