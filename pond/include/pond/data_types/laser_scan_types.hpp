#pragma once
#include "stamp.hpp"
#include <memory>
#include <vector>

struct LaserScan
{
    Stamp stamp;
    float angle_min;
    float angle_max;
    float angle_increment;
    float time_increment;
    float scan_time;
    float range_min;
    float range_max;
    std::vector<float> ranges;
    std::vector<float> intensities;
};

using LaserScanSPtr = std::shared_ptr<LaserScan>;
