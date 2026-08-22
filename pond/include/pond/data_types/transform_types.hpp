#pragma once

#include "stamp.hpp"

struct FrameTransform
{
    Stamp stamp;
    std::string parent_frame_id;

    Sophus::SE3d tf;
};

