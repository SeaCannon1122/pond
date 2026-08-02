#include "pond_manager.hpp"
#include <chrono>

int main()
{
    printf("start\n"); fflush(stdout);
    {
        PondManager pm;

        pm.load_module("usb_cam_module", "quac_modules", "usb_cam_driver", "default_thread");
        pm.load_module("gst_module", "quac_modules", "gst_server", "default_thread");
        
        std::this_thread::sleep_for(std::chrono::seconds(3));
    
        pm.unload_module("gst_module");
        pm.unload_module("usb_cam_module");
    }
    printf("end\n"); fflush(stdout);
}