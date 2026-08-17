#include "pond/pond.h"
#include "manager/pond_manager.hpp"
#include <chrono>

#include <atomic>
#include <signal.h>

std::atomic<bool> is_running{true};
int32_t int_counter = 0;

void signalHandler(int signal)
{
    if (signal != SIGINT && signal != SIGTERM)return;
    printf(" INTERRUPT\n"); fflush(stdout);
    is_running.store(false);

    int_counter++;
    if (int_counter == 10) exit(187);
}

int main()
{
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    {
        PondManager pm;

        pm.load_module("camera", "quac_modules", "dummy_camera", "default_thread", 
            {
                {"width", pond_malloc_parameter_int(1920)},
                {"height", pond_malloc_parameter_int(1080)},
                {"fps", pond_malloc_parameter_int(30)},
            },
            {{"out", "image"}}
        );
        pm.load_module("streamer", "quac_modules", "gst_server", "default_thread",
            {
                {"width", pond_malloc_parameter_int(1920)},
                {"height", pond_malloc_parameter_int(1080)},
                {"port", pond_malloc_parameter_int(5000)},
                {"ip", pond_malloc_parameter_string((uint8_t*)"127.0.0.1")},
                {"bitrate", pond_malloc_parameter_int(10000)},
                {"format", pond_malloc_parameter_string((uint8_t*)"RGB8")},
            },
            {{"in", "image"}}
        );
        
        while (is_running.load()) std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
        pm.shutdown_module("streamer");
        pm.shutdown_module("camera");
    }
}