#pragma once

#include "stamp.hpp"

struct FrameTransform
{
    Stamp stamp;
    std::string child_frame_id;
    Sophus::SE3d tf;
};

