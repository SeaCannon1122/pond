#pragma once

#include "stamp.hpp"

struct MotorCommand
{
    double pos = 0;
    double vel = 0;
    double torque = 0;
};

struct MotorFeedback
{
    double time = 0;
    double hw_time = 0;
    double pos = 0;
    double vel = 0;
    double current = 0;
    double temperature = 0;
};