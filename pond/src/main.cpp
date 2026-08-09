#include "pond_manager.hpp"
#include <chrono>

#include <atomic>
#include <signal.h>

std::atomic<bool> is_running{true};

void signalHandler(int signal)
{
    if (signal != SIGINT && signal != SIGTERM)return;
    printf(" INTERRUPT\n"); fflush(stdout);
    is_running.store(false);
}

int main()
{
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    {
        PondManager pm;

        pm.load_module("usb_cam_module", "quac_modules", "usb_cam_driver", "default_thread");
        pm.load_module("gst_module", "quac_modules", "gst_server", "default_thread");
        
        while (is_running.load()) std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
        pm.shutdown_module("gst_module");
        pm.shutdown_module("usb_cam_module");
    }
}