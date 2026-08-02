#include <pond/pond.hpp>

int demo_module_startup()
{


    return 0;
}

void demo_module_shutdown()
{

}

void demo_module_frame()
{

}


POND_HPP_DECLARE_MODULE(demo_module_startup, demo_module_shutdown, demo_module_frame)