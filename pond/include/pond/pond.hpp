#pragma once

#include "hpp/module_base.hpp"

namespace pond
{
    inline double get_time() {return pond_get_time();}
    inline double get_monotonic_time() {return pond_get_monotonic_time();}
    inline void sleep(double duration) {return pond_sleep(duration);}
}
