#include "stamp.hpp"

struct MotorCommand
{
    Stamp stamp;
    double pos = 0;
    double vel = 0;
    double torque = 0;
};

struct MotorFeedback
{
    Stamp stamp;
    double pos = 0;
    double vel = 0;
    double current = 0;
    double temperature = 0;
};